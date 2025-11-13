from io import BytesIO

import numpy as np


def recv_frame(sock, bufsize: int = 1024) -> np.ndarray:  # type: ignore[override]
    length = None
    frame_buffer = bytearray()
    while True:
        data = sock.recv(bufsize)
        if len(data) == 0:
            return np.array([])
        frame_buffer += data
        if len(frame_buffer) == length:
            break
        while True:
            if length is None:
                if b":" not in frame_buffer:
                    break
                length_str, ignored, frame_buffer = frame_buffer.partition(b":")
                length = int(length_str)
            if len(frame_buffer) < length:
                break

            frame_buffer = frame_buffer[length:]
            length = None
            break

    frame = np.load(BytesIO(frame_buffer), allow_pickle=True)["frame"]
    return frame


def pack_frame(frame: np.ndarray) -> bytearray:
    f = BytesIO()
    np.savez(f, frame=frame)

    packet_size = len(f.getvalue())
    header = "{0}:".format(packet_size)
    header_bytes = bytes(header.encode())  # prepend length of array

    out = bytearray()
    out += header_bytes

    f.seek(0)
    out += f.read()
    return out
