"""Bounded, lossless PointCloud2 wire format; no geometric interpretation."""
import base64
import binascii
from .protocol import INTEGER_TYPES, STRING_TYPES, wire

MAX_CLOUD_BYTES = 900000
KEYS = {"header", "height", "width", "fields", "is_bigendian", "point_step", "row_step", "is_dense", "data_b64"}


def validate_layout(value):
    for key in ("height", "width", "point_step", "row_step"):
        if type(value.get(key)) not in INTEGER_TYPES or value[key] <= 0:
            raise ValueError("invalid_cloud_layout:"+key)
    if (not 18 <= value["point_step"] <= 256 or value["height"]*value["width"] > 100000 or
            value["row_step"] < value["width"]*value["point_step"] or
            value["row_step"]*value["height"] > MAX_CLOUD_BYTES):
        raise ValueError("cloud_layout_too_large")
    if any(type(value.get(key)) is not bool for key in ("is_bigendian", "is_dense")):
        raise ValueError("invalid_cloud_flags")
    fields = value.get("fields")
    if not isinstance(fields, list) or not 1 <= len(fields) <= 32:
        raise ValueError("invalid_cloud_fields")
    names = set()
    for field in fields:
        if (not isinstance(field, dict) or set(field) != {"name", "offset", "datatype", "count"} or
                not isinstance(field["name"], STRING_TYPES) or not 1 <= len(field["name"]) <= 64 or
                field["name"] in names or type(field["offset"]) not in INTEGER_TYPES or
                not 0 <= field["offset"] < value["point_step"] or
                type(field["datatype"]) not in INTEGER_TYPES or not 1 <= field["datatype"] <= 8 or
                type(field["count"]) not in INTEGER_TYPES or not 1 <= field["count"] <= 16):
            raise ValueError("invalid_cloud_field")
        names.add(field["name"])


def encode_cloud(message):
    value = dict(header=wire(message.header), height=message.height, width=message.width,
        fields=wire(message.fields), is_bigendian=message.is_bigendian,
        point_step=message.point_step, row_step=message.row_step, is_dense=message.is_dense)
    validate_layout(value)
    raw = bytes(bytearray(message.data))
    if len(raw) != message.row_step*message.height:
        raise ValueError("invalid_cloud_data_length")
    value["data_b64"] = base64.b64encode(raw).decode("ascii")
    return value


def decode_cloud(value):
    if not isinstance(value, dict) or set(value) != KEYS:
        raise ValueError("invalid_cloud_packet")
    validate_layout(value)
    text = value["data_b64"]
    if not isinstance(text, STRING_TYPES) or len(text) > 4*((MAX_CLOUD_BYTES+2)//3):
        raise ValueError("cloud_payload_too_large")
    try:
        raw = base64.b64decode(text.encode("ascii"), validate=True)
    except (ValueError, UnicodeError, binascii.Error):
        raise ValueError("invalid_cloud_base64")
    if len(raw) != value["row_step"]*value["height"]:
        raise ValueError("invalid_cloud_data_length")
    return dict((key, item) for key, item in value.items() if key != "data_b64"), raw
