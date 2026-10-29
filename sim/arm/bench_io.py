"""The benches' saved trials: one pickled record appended per trial (a whole list rewritten each trial grew slow at full
resolution, 29 Sep), or the older single list. Only the benches' own files, written on this machine, are read."""
import pickle


def load(path):
    """every trial record in a bench file, either format"""
    out = []
    with open(path, "rb") as f:
        while True:
            try: x = pickle.load(f)
            except EOFError: break
            out.extend(x if isinstance(x, list) else [x])
    return out


def start(path):
    """an empty bench file"""
    open(path, "wb").close()


def append(path, record):
    """one more trial"""
    with open(path, "ab") as f: pickle.dump(record, f)
