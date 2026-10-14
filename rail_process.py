"""Process-isolated fitting with a latest-only mailbox and nonblocking UI reads.

Protocol-2 pickle is used ONLY for files this application creates in its private
temporary directory, never for PCAPs or user-supplied serialized objects.
"""
import json
import os
try:
    import cPickle as pickle  # Avoid pure-Python serialization in RSView 2.7.
except ImportError:
    import pickle
import shutil
import subprocess
import sys
import tempfile
import time
from collections import deque


def atomic_write(path, value):
    with open(path + '.tmp', 'wb') as stream:
        pickle.dump(value, stream, 2)
    os.rename(path + '.tmp', path)


def read_object(path):
    with open(path, 'rb') as stream:
        return pickle.load(stream)


def point_count(points):
    return points['count'] if isinstance(points, dict) else len(points)


def remove_file(path):
    try:
        os.unlink(path)
    except OSError:
        if os.path.exists(path):
            raise


class ProcessWorker(object):
    def __init__(self, python=None):
        if python is None:
            if sys.version_info[0] >= 3:
                # Windows venv python.exe can be a launcher: track the actual
                # interpreter, not a parent that exits before its child closes.
                python = getattr(sys, '_base_executable', sys.executable)
            else:
                with open(os.path.join(os.path.dirname(__file__), 'worker_config.json')) as stream:
                    python = json.load(stream)['python']
        if not os.path.isfile(python):
            raise RuntimeError('Calculation Python not found: %s' % python)
        self.directory = tempfile.mkdtemp(prefix='rsview-rail-')
        self.pending = self.running = self.result = self.last_job = None
        self.events = deque(maxlen=256)
        self.lost_events = 0
        self.closed = self.death_reported = False
        self.error_stream = open(os.path.join(self.directory, 'worker-error.log'), 'w+b')
        env = os.environ.copy()
        for name in ('PYTHONHOME', 'PYTHONPATH', 'PYTHONSTARTUP'):
            env.pop(name, None)  # RSView's Python 2 environment must not leak into Python 3.
        # Python 2 may set __file__ to its .pyc on subsequent RSView starts.
        # Python 3 must execute source, never that incompatible bytecode cache.
        script = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rail_process.py')
        args = [python, script, self.directory, str(os.getpid())]
        if sys.version_info[0] == 2 and os.name == 'nt':
            # The embedded Python 2 mixes JSON unicode and filesystem MBCS paths.
            args = [a.encode('mbcs') if isinstance(a, unicode) else a for a in args]
        try:
            self.child = subprocess.Popen(
                args,
                stdout=self.error_stream, stderr=self.error_stream, env=env,
                creationflags=0x08000000 if os.name == 'nt' else 0)
        except Exception:
            self.error_stream.close()
            shutil.rmtree(self.directory, ignore_errors=True)
            raise

    def _record(self, event, **fields):
        if len(self.events) == self.events.maxlen:
            self.lost_events += 1
        fields.update(event=event, time=time.time())
        self.events.append(fields)

    def _path(self, sequence, suffix):
        return os.path.join(self.directory, '%012d.%s' % (sequence, suffix))

    def submit(self, job):
        if self.closed:
            return
        self._pump()
        if self.pending:
            path = self._path(self.pending['sequence'], 'request')
            if os.path.exists(path):
                remove_file(path)
                self._record('pending_replaced', **self.pending)
        self.pending = dict(sequence=job[0], received=job[1], point_count=point_count(job[2]))
        self.last_job = dict(self.pending)
        atomic_write(self._path(job[0], 'request'), job)
        self._record('submitted', **self.pending)

    def _pump(self):
        if self.closed:
            return
        names = os.listdir(self.directory)
        for name in sorted(n for n in names if n.endswith('.started')):
            path = os.path.join(self.directory, name)
            self.running = read_object(path)
            remove_file(path)
            if self.pending and self.pending['sequence'] == self.running['sequence']:
                self.pending = None
            self._record('started', **self.running)
        for name in sorted(n for n in names if n.endswith('.result')):
            path = os.path.join(self.directory, name)
            result, info = read_object(path)
            remove_file(path)
            self._record('completed', **info)
            if self.running and self.running['sequence'] == result[0]:
                self.running = None
            if self.result:
                self._record('result_replaced', sequence=self.result[0])
            self.result = result
        if self.child.poll() is not None and not self.death_reported:
            self.death_reported = True
            self.error_stream.seek(0)
            detail = self.error_stream.read(2000).decode('utf-8', 'replace')
            error = 'Calculation process exited (%s): %s' % (self.child.returncode, detail)
            self._record('worker_exited', error=error)
            if self.last_job:
                self.result = (self.last_job['sequence'], self.last_job['received'], None, 0., error)

    def poll(self):
        self._pump()
        result, self.result = self.result, None
        return result

    def diagnostics(self):
        self._pump()
        state = dict(running=self.running, pending=self.pending,
                     alive=not self.closed and self.child.poll() is None,
                     lost_events=self.lost_events,
                     worker_location='analyze_live (separate process)')
        events = list(self.events)
        self.events.clear()
        return state, events

    def cancel_pending(self):
        if self.closed:
            return
        self._pump()
        for name in os.listdir(self.directory):
            if name.endswith('.request'):
                remove_file(os.path.join(self.directory, name))
        if self.pending:
            self._record('pending_cancelled', **self.pending)
        if self.result:
            self._record('result_cancelled', sequence=self.result[0])
        self.pending = self.result = None
        # A claimed task may finish, but overlay.accept_after rejects its result.

    def close(self):
        if self.closed:
            return
        self.cancel_pending()
        self.closed = True
        if self.child.poll() is None:
            self.child.terminate()  # Only the process spawned by this worker.
        self.child.wait()
        self.error_stream.close()
        target = os.path.realpath(self.directory)
        base = os.path.realpath(tempfile.gettempdir())
        if os.path.dirname(target) != base or not os.path.basename(target).startswith('rsview-rail-'):
            raise RuntimeError('Refusing cleanup outside owned temporary directory')
        shutil.rmtree(target)


def run(directory, parent_pid):
    from rail_live import analyze_live
    parent_handle = None
    if os.name == 'nt':
        import ctypes
        kernel = ctypes.windll.kernel32
        kernel.OpenProcess.restype = ctypes.c_void_p
        parent_handle = kernel.OpenProcess(0x00100000, False, parent_pid)
        if not parent_handle:
            return
        kernel.WaitForSingleObject.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        kernel.CloseHandle.argtypes = [ctypes.c_void_p]
    try:
        while os.path.isdir(directory):
            if parent_handle:
                if kernel.WaitForSingleObject(parent_handle, 0) != 258:
                    return
            elif os.getppid() != parent_pid:
                return
            requests = sorted(n for n in os.listdir(directory) if n.endswith('.request'))
            if not requests:
                time.sleep(.005)
                continue
            newest = requests[-1]
            for old in requests[:-1]:
                remove_file(os.path.join(directory, old))
            source = os.path.join(directory, newest)
            claimed = source + '.working'
            try:
                os.rename(source, claimed)
            except OSError:
                continue  # The UI replaced/cancelled this pending task.
            sequence, received, points = read_object(claimed)
            remove_file(claimed)
            started = time.time()
            info = dict(sequence=sequence, received=received, point_count=point_count(points),
                        started=started, worker_pid=os.getpid())
            atomic_write(os.path.join(directory, '%012d.started' % sequence), info)
            error, analysis = '', None
            try:
                analysis = analyze_live(points)
            except Exception as exc:
                error = str(exc)
            finished = time.time()
            info.update(finished=finished, compute_ms=(finished - started) * 1000.,
                        queue_ms=(started - received) * 1000., error=error)
            result = (sequence, received, analysis, info['compute_ms'], error)
            atomic_write(os.path.join(directory, '%012d.result' % sequence), (result, info))
    finally:
        if parent_handle:
            kernel.CloseHandle(parent_handle)


if __name__ == '__main__':
    run(sys.argv[1], int(sys.argv[2]))
