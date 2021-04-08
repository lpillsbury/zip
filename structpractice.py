'''

# TELEMETRY_STRUCT = struct.Struct(">Hhffb31B")
command_form = ">fB3s"
telem_form = ">Hhffb31B"
print(zs.COMMAND_STRUCT.size)
print(zs.TELEMETRY_STRUCT.size)
var = zs.COMMAND_STRUCT.pack(23, 1, bytes(3))
print(zs.COMMAND_STRUCT)
print(var)
var2 = struct.unpack(command_form,var)
print(var2)
print("\n")


import struct
var = struct.pack('hhl', 5, 10, 15)
print(var)
print(struct.unpack('hhl', var))
'''
