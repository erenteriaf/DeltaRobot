import snap7
from snap7.util import *

client = snap7.client.Client()
client.connect("192.168.0.1", 0, 1)
def write_oef_plc(x, y):
    DB_NUMBER = 6
    START     = 0              # first byte in the DB
    SIZE      = 12             # <—- change to the number of bytes you actually need

    o_ef_plc = client.db_read(DB_NUMBER, START, SIZE)  # bytearray with your data

    set_real(o_ef_plc, 0, x)  # set REAL at byte 0
    set_real(o_ef_plc, 4, y)  # set REAL at byte 4
    client.db_write(DB_NUMBER, START, o_ef_plc)  # write back to PLC

def write_oef(x, y, z):
    DB_NUMBER = 6
    START     = 92              # first byte in the DB
    SIZE      = 12             # <—- change to the number of bytes you actually need

    o_ef_plc = client.db_read(DB_NUMBER, START, SIZE)  # bytearray with your data

    set_real(o_ef_plc, 0, x)  # set REAL at byte 0
    set_real(o_ef_plc, 4, y)  # set REAL at byte 4
    set_real(o_ef_plc, 8, z)  # set REAL at byte 8
    client.db_write(DB_NUMBER, START, o_ef_plc)  # write back to PLC

def read_my_turn():
    DB_NUMBER = 6
    START     = 116              # first byte in the DB
    SIZE      = 1             # <—- change to the number of bytes you actually need

    my_turn = client.db_read(DB_NUMBER, START, SIZE)  # bytearray with your data

    my_turn_value = get_bool(my_turn, 0, 0)  # get BOOL at byte 0, bit 0
    return my_turn_value

def read_my_turn_memory():
    DB_NUMBER = 6
    START     = 116              # first byte in the DB
    SIZE      = 1             # <—- change to the number of bytes you actually need

    my_turn = client.db_read(DB_NUMBER, START, SIZE)  # bytearray with your data

    my_turn_value = get_bool(my_turn, 0, 0)  # get BOOL at byte 0, bit 0
    return my_turn_value

def write_token_plc(token_x, token_y, token_z):
    DB_NUMBER = 6
    START     = 104             # first byte in the DB
    SIZE      = 12             # <—- change to the number of bytes you actually need

    o_ef_plc = client.db_read(DB_NUMBER, START, SIZE)  # bytearray with your data

    set_real(o_ef_plc, 0, token_x)  # set REAL at byte 0
    set_real(o_ef_plc, 4, token_y)  # set REAL at byte 4
    set_real(o_ef_plc, 8, token_z)  # set REAL at byte 8
    client.db_write(DB_NUMBER, START, o_ef_plc)  # write back to PLC

def go_home():
    DB_NUMBER = 6
    START     = 117          # first byte in the DB
    SIZE      = 1             # <—- change to the number of bytes you actually need

    home = client.db_read(DB_NUMBER, START, SIZE)  # bytearray with your data

    set_bool(home, 0, 3, 1)  # set REAL at byte 0
    client.db_write(DB_NUMBER, START, home)  # write back to PLC

def go_out_home():
    DB_NUMBER = 6
    START     = 134          # first byte in the DB
    SIZE      = 1             # <—- change to the number of bytes you actually need

    home = client.db_read(DB_NUMBER, START, SIZE)  # bytearray with your data

    set_bool(home, 0, 0, 1)  # set REAL at byte 0
    client.db_write(DB_NUMBER, START, home)  # write back to PLC

def start_play():
    DB_NUMBER = 6
    START     = 117          # first byte in the DB
    SIZE      = 1             # <—- change to the number of bytes you actually need

    start_play = client.db_read(DB_NUMBER, START, SIZE)  # bytearray with your data

    set_bool(start_play, 0, 1, 1)  # set BOOL at byte 0, bit 0
    client.db_write(DB_NUMBER, START, start_play)  # write back to PLC

def close_gripper():
    DB_NUMBER = 6
    START     = 40          # first byte in the DB
    SIZE      = 1             # <—- change to the number of bytes you actually need

    gripper = client.db_read(DB_NUMBER, START, SIZE)  # bytearray with your data

    set_bool(gripper, 0, 1, 1)  # set BOOL at byte 0, bit 0
    client.db_write(DB_NUMBER, START, gripper)  # write back to PLC

def open_gripper():
    DB_NUMBER = 6
    START     = 40          # first byte in the DB
    SIZE      = 1             # <—- change to the number of bytes you actually need

    gripper = client.db_read(DB_NUMBER, START, SIZE)  # bytearray with your data

    set_bool(gripper, 0, 1, 0)  # set BOOL at byte 0, bit 0
    client.db_write(DB_NUMBER, START, gripper)  # write back to PLC

def activate_conveyor():
    DB_NUMBER = 6
    START     = 134          # first byte in the DB
    SIZE      = 1             # <—- change to the number of bytes you actually need

    conveyor = client.db_read(DB_NUMBER, START, SIZE)  # bytearray with your data

    set_bool(conveyor, 0, 1, 1)  # set BOOL at byte 0, bit 0
    client.db_write(DB_NUMBER, START, conveyor)  # write back to PLC

def deactivate_conveyor():
    DB_NUMBER = 6
    START     = 134          # first byte in the DB
    SIZE      = 1             # <—- change to the number of bytes you actually need

    conveyor = client.db_read(DB_NUMBER, START, SIZE)  # bytearray with your data

    set_bool(conveyor, 0, 1, 0)  # set BOOL at byte 0, bit 0
    client.db_write(DB_NUMBER, START, conveyor)  # write back to PLC

def activate_magnet():
    DB_NUMBER = 6
    START     = 134          # first byte in the DB
    SIZE      = 1             # <—- change to the number of bytes you actually need

    conveyor = client.db_read(DB_NUMBER, START, SIZE)  # bytearray with your data

    set_bool(conveyor, 0, 2, 1)  # set BOOL at byte 0, bit 0
    client.db_write(DB_NUMBER, START, conveyor)  # write back to PLC

def deactivate_magnet():
    DB_NUMBER = 6
    START     = 134          # first byte in the DB
    SIZE      = 1             # <—- change to the number of bytes you actually need

    conveyor = client.db_read(DB_NUMBER, START, SIZE)  # bytearray with your data

    set_bool(conveyor, 0, 2, 0)  # set BOOL at byte 0, bit 0
    client.db_write(DB_NUMBER, START, conveyor)  # write back to PLC