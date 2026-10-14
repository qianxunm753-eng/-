# -*- coding: utf-8 -*-
"""Python2/3-safe selected PCAP input; no silent shared-record fallback."""
import os
import sys
import tempfile

try:
    text_type=unicode
except NameError:
    text_type=str

_MAGIC=(b'\xd4\xc3\xb2\xa1',b'\xa1\xb2\xc3\xd4',b'\x4d\x3c\xb2\xa1',b'\xa1\xb2\x3c\x4d',b'\x0a\x0d\x0d\x0a')


def _path_text(value):
    if isinstance(value,text_type):return value
    if not isinstance(value,bytes):return text_type(value)
    candidates=[]
    for encoding in ('utf-8','mbcs' if os.name=='nt' else sys.getfilesystemencoding()):
        try:decoded=value.decode(encoding)
        except UnicodeError:continue
        if decoded not in candidates:candidates.append(decoded)
    existing=[path for path in candidates if os.path.isfile(path)]
    if len(existing)>1:
        raise ValueError('ambiguous PCAP path encoding; select the file explicitly')
    if existing:return existing[0]
    if candidates:return candidates[0]
    raise ValueError('PCAP path encoding could not be decoded')


def choose_current_pcap(logic):
    """The missing legacy pointer cannot tell us the intended file: ask again."""
    default=os.path.join(os.path.expanduser('~'),'Desktop')
    return logic.QtGui.QFileDialog.getOpenFileName(logic.getMainWindow(),
        'Confirm PCAP file (legacy path record not trusted)',default,'PCAP files (*.pcap)')


def _selected_source(input_path,choose_file,logic):
    path=_path_text(input_path)
    # The known shared filename.tmp.log may exist while still naming an old
    # selection. Its existence is not proof of the current GUI user's intent.
    shared_record=os.path.basename(path).lower()=='filename.tmp.log'
    if shared_record or not os.path.isfile(path):
        value=choose_file(logic)
        if isinstance(value,(tuple,list)):value=value[0]
        if not value:return None
        source=_path_text(value)
    else:
        with open(path,'rb') as stream:
            head=stream.read(4)
            if head in _MAGIC:
                source=path
            else:
                content=head+stream.read(65533)
                if len(content)>65536:
                    raise ValueError('PCAP path record exceeds the safe text bound')
                source=_path_text(content.strip())
    source=os.path.abspath(source)
    if not os.path.isfile(source) or not source.lower().endswith('.pcap'):
        raise ValueError('Selected PCAP file does not exist or is not a PCAP')
    return source


def open_pcap_input(logic,native_open,input_path,positionFilename=None,choose_file=None):
    """Translate only the current explicit input into the existing native loader.

    A real capture and an existing pointer remain supported. Missing metadata
    triggers explicit selection/cancel, never a different global old PCAP.
    Only a private small pointer is written, not another capture/point cloud.
    """
    source=_selected_source(input_path,choose_file or choose_current_pcap,logic)
    if source is None:return None
    encoding='mbcs' if os.name=='nt' else sys.getfilesystemencoding()
    source_bytes=source.encode(encoding)  # This installed Windows native reader uses ANSI paths.
    folder=tempfile.mkdtemp(prefix='rsview-open-selected-')
    pointer=os.path.join(folder,'input_path.log')
    try:
        with open(pointer,'wb') as stream:stream.write(source_bytes)
        result=native_open(pointer,positionFilename)
        if logic.getReader() is not None:
            loaded=_path_text(logic.getReaderFileName())
            if os.path.normcase(os.path.abspath(loaded))!=os.path.normcase(source):
                closer=getattr(logic,'close',None)
                if closer is not None:closer()
                raise ValueError('Native loader opened a different PCAP than the current selection')
        return result
    finally:
        # Only these three files can be ours; retain any unexpected contents.
        for name in ('input_path.log','tmp.log','input.log'):
            owned=os.path.join(folder,name)
            if os.path.isfile(owned):os.remove(owned)
        try:os.rmdir(folder)
        except OSError:pass
