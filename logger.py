import time
import argparse
import logging
import serial
import json

com_port = ""


def write_expect(ser, tx, expect):
        ser.write(tx + b'\r')
        ser.readline()

        read = ser.readline()[:-2]

        if read != expect:
            print(f"error: expect {tx}, got {read}")
            return False
        return True

def parse_samples(ser, len):
    samples1, samples2, samples3 = [], [], []
    timestamps = []
    ids = []
    group_ids = []

    group_id = -1
    last_id = -1

    for _ in range(len):
        id_line = ser.readline().decode("utf-8").strip()
        sample1_line = ser.readline().decode("utf-8").strip()
        sample2_line = ser.readline().decode("utf-8").strip()
        sample3_line = ser.readline().decode("utf-8").strip()

        assert id_line[0] == '#'

        _id = int(id_line[1:])
        if _id < last_id or last_id == -1:
            timestamps.append(time.time())
            group_id += 1

        last_id = _id

        if group_id >= 0:
            group_ids.append(group_id)
            ids.append(_id)
            samples1.append(int(sample1_line))
            samples2.append(int(sample2_line))
            samples3.append(int(sample3_line))

    return ids, group_ids, timestamps, samples1, samples2, samples3

def interpolate_time(group_ids, ids, timestamps):
    group_size = sum([d == 0 for d in group_ids])
    ts = []

    for i in range(len(ids)):
        gid = group_ids[i]
        if gid + 1 == len(timestamps):
            break

        id_ = ids[i]
        t = timestamps[gid] + (timestamps[gid+1] - timestamps[gid]) * id_ / group_size


        ts.append(t)

    return ts

if __name__ == "__main__":
    logging.basicConfig(format='%(asctime)s %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p', level=logging.INFO)

    parser = argparse.ArgumentParser(description="Serial logger for pio_adc program")

    parser.add_argument("com", metavar='COM', type=str,
            help="COM Port connecting to the MCU.")
    parser.add_argument("len", metavar='LEN', type=int,
            help="Number of measurements to log")
    parser.add_argument("output", metavar='OUTPUT', type=str,
            help="Output file name")
    parser.add_argument('-f', '--fast', action='store_true', help="Fast sampling mode (1 kHz)")
    args = parser.parse_args()

    com_port = args.com

    ser = serial.Serial(com_port, 115200, timeout=5)

    try:
        ser.flush()
        ser.write(b'stop\r')

        time.sleep(0.01)

        while ser.in_waiting:
            ser.read(1)

        ser.write(b'ping\r')

        time.sleep(0.01)

        ok = False

        while ser.in_waiting:
            r = ser.readline()
            if r == b'pong\r\n':
                ok = True
                break

        if not ok:
            print("invalid/no ping response")
            exit()

        assert write_expect(ser, b'ping', b'pong')
        assert write_expect(ser, b'unset_verbose', b'unset verbose')
        if args.fast:
            assert write_expect(ser, b'set_fast', b'set fast')
        else:
            assert write_expect(ser, b'set_slow', b'set slow')

        assert write_expect(ser, b'start', b'started')

        ids, group_ids, timestamps, samples1, samples2, samples3 = parse_samples(ser, args.len)

        ser.write(b'stop\r')

        ts = interpolate_time(group_ids, ids, timestamps)

        with open(args.output, 'w') as f:
            json.dump({
                'ts': ts,
                'ids': ids[:len(ts)],
                'group_ids': group_ids[:len(ts)],
                'samples1': samples1[:len(ts)],
                'samples2': samples2[:len(ts)],
                'samples3': samples3[:len(ts)],
                }, f)


    finally:
        ser.close()
