#!/usr/bin/env python3

import sys

from rqt_gui.main import Main


def main():
    sys.exit(Main().main(sys.argv, standalone='bica_rqt_graph.bica_graph.BicaGraph'))


if __name__ == '__main__':
    main()
