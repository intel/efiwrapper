#!/usr/bin/env python

import argparse
import os
import struct

import CRC

#
#  TODO:
#  - documentation: Add ArgumentParser description, epilog
#  - error handling!
#

#========================================================================
# KP image creation
#========================================================================

MAGIC  = 0x2E6B7069
HEADER = struct.Struct('IIIIIII')


def image (type, payload, ext_hdr=''):

    bytes = bytearray (HEADER.size + len(ext_hdr) + len(payload) + 4)

    version = 0
    length  = len(payload)
    offset  = HEADER.size + len(ext_hdr)
    uncomp  = length	## compression not supported (yet)
    hcrc    = 0
    HEADER.pack_into (bytes, 0,
                      MAGIC, type, version, length, offset, uncomp, hcrc)
    crc = CRC.crc32c_buf (bytes[0:24])
    struct.pack_into ('I', bytes, 24, crc)

    bytes[HEADER.size:offset] = ext_hdr
    bytes[offset:offset+length] = payload
    crc = CRC.crc32c_buf(bytes[HEADER.size:offset+length])
    struct.pack_into ('I', bytes, offset+length, crc)

    return bytes


def multi_image (type, files):

    nfile = len(files)

    subimg = []
    o = 0
    for f in files:
        l = len(f)
        subimg.append((o, l))
        o += round_up(l)

    sizes   = bytearray(4*nfile)
    payload = bytearray(o)
    for i in range(nfile):
        o, l = subimg[i]
        struct.pack_into ('I', sizes, 4*i, l)
        payload[o:o+l] = files[i]

    return image (type, payload, sizes)

# ========================================================================
# Support functions.
# ========================================================================

def round_up (value, div=4):
    """Round VALUE up to the next multiple of DIV (a power of two)."""
    return (value + div - 1) & ~(div - 1)

def write_part (basename, partname, image, o=None, l=None):
    """Write a part of the image to a file."""
    with open (basename + '.' + partname + '.bin', 'wb') as file:
        file.write (image[o:o+l] if o else image)

#========================================================================
# Main program: Argument handling.
#========================================================================

def parse_args():
    """Parse the command line and return an argparse.Namespace object."""

    parser = argparse.ArgumentParser()  ## TODO: description='...', epilog='...')

    parser.add_argument ('-V', '--version',
                         action='version', version='%(prog)s 0.1')

    parser.add_argument ('-v', '--verbose',
                         action='count',
                         help='increase verbosity')

    parser.add_argument ('-m', '--multi-file',
                         help='create a multi-file image',
                         action="store_true")
    parser.add_argument ('-o', '--output',
                         default='image.bin',
                         help='write final image to OUTPUT')
    parser.add_argument ('-t', '--type',
                         help='set image TYPE (in hex)',
                         default=0,
                         type=lambda s: int(s, 16))
    parser.add_argument ('file',
                         help='add FILE to payload of IMAGE being created',
                         nargs='+',
                         metavar='file')

    return parser.parse_args()

# ------------------------------------------------------------------------

def main():

    a = parse_args()

    #print a

    data = [open(f, 'rb').read() for f in a.file]
    if a.multi_file or len(data) != 1:
        img = multi_image(a.type, data)
    else:
        img = image(a.type, data[0])
    open (a.output, 'wb').write (img)

if __name__ == '__main__':
    main()

# ========================================================================
