#!/usr/bin/env python
'''
*******************************************************
 Copyright (c) MLRS project
 GPL3
 https://www.gnu.org/licenses/gpl-3.0.de.html
*******************************************************
 python script to generate rp-uartX.h librarires
********************************************************
'''
import re
import datetime


templatefname = "rp-uart-template.h"
    
F = open(templatefname, "r")
templatecode = F.read()
F.close()

def generate(code, periphname):
    outfname = "rp-"+periphname+".h"
    print('generate', outfname)
    code1 = code.replace('UART$',periphname.upper())
    code2 = code1.replace('uart$',periphname)
    F = open(outfname, "w")
    F.write(code2)
    F.close() 

generate(templatecode, 'uart')
generate(templatecode, 'uartb')
generate(templatecode, 'uartc')
generate(templatecode, 'uartd')
generate(templatecode, 'uarte')
generate(templatecode, 'uartf')
