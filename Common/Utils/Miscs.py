import yaml


def ReadYaml(filepath):
    with open(filepath, encoding="utf-8", mode='r') as f:
        result = yaml.load(stream=f, Loader=yaml.FullLoader)
        return result


def WriteYaml(filepath, data):
    with open(filepath, encoding="utf-8", mode='w') as f:
        yaml.dump(data, stream=f, allow_unicode=True)


def Ascii2Hex(data_h, data_l):
    if ord(data_h) >= ord('A'):
        # 'A' to 'F'
        output_h = ord(data_h) - ord('A') + 10
    else:
        # '0' to '9'
        output_h = ord(data_h) - 48

    if ord(data_l) >= ord('A'):
        output_l = ord(data_l) - ord('A') + 10
    else:
        output_l = ord(data_l) - 48

    output = output_h << 4 | output_l
    return output


def Hex2Ascii(data):
    data_h = data >> 4
    data_l = data - (data_h << 4)

    if int(data_h) > 9:
        output_h = int(data_h) - 10 + 65
    else:
        output_h = int(data_h) + 48

    if int(data_l) > 9:
        output_l = int(data_l) - 10 + 65
    else:
        output_l = int(data_l) + 48

    return chr(output_h), chr(output_l)


def DecodeBytes(decodeStr: bytes):
    """ Use for decode serial or net string."""
    try:
        unicodeStr = decodeStr.decode()
    except:
        unicodeStr = "".join(map(chr, decodeStr))

    return unicodeStr


__all__ = ["Ascii2Hex", "Hex2Ascii", "DecodeBytes", "WriteYaml", "ReadYaml"]
