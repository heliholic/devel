#!/usr/bin/python3

import sys

def main(args):
    release = args[1]
    inputfile = args[2]
    outputfile = args[3]

    with open(inputfile) as input:
        with open(outputfile, 'w') as output:
            for line in input:
                if line.strip() == '# ' + release:
                    break
            for line in input:
                if line.strip() == '***':
                    break
                output.write(line)


if __name__ == '__main__': main(sys.argv)
