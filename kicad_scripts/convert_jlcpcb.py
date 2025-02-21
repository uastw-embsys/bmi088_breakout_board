#!/usr/bin/env python3
# ----------------------------------------------------------------------
# Author: Christian Fibich <fibich@technikum-wien.at>
# 
# Stadt Wien Kompetenzteam für Drohnentechnik in der Fachhochschulausbildung
# Kompetenzfeld Embedded Systems
# Fachhochschule Technikum Wien
#
# All rights reserved.
#
# Date: 29.1.2025
# ----------------------------------------------------------------------

import argparse
import csv

# Generate from Kicad:
# BOM (File -> Fabrication Outputs -> Bill Of Materials) as CSV
# POS (File -> Fabrication Outputs ->  Component Placements:
#                   - Format: CSV
#                   - Units: Millimeters
#                   - Files: Single file for board
#                   [x] Include only SMD footprints
#                   [x] Exclude all footprints with through hole pads
#                   [x] Use drill/place file origin
# Then launch
#   $ python3 convert_jlcpcb.py BOM POS
# This will generate two files: BOM.csv and POS.csv with column names and order accepted by JLCPCB

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('BOM')
    parser.add_argument('POS')
    parser.add_argument('--filter-designator',action='append')
    args = parser.parse_args()
    
    if args.filter_designator is not None:
        filter_components = set(args.filter_designator)
    else:
        filter_components = set()
    
    with open(args.BOM,'r') as f:
        bomreader = csv.DictReader(f,delimiter=';',quotechar='"')
        lines = []
        remap = {'Comment': 'Designation', 'Designator': 'Designator','Footprint': 'Footprint'}
        
        for line in bomreader:
            if line['Designator'] in filter_components:
                continue
        
            lines.append({k1: line[k0] for k1, k0 in remap.items()})
            
        with open('BOM.csv','w') as f:
            poswriter = csv.DictWriter(f,delimiter=',',quotechar='"',fieldnames=remap.keys())
            poswriter.writeheader()
            for line in lines:
                poswriter.writerow(line)
            
                
    with open(args.POS,'r',newline='') as f:
        posreader = csv.DictReader(f,delimiter=',',quotechar='"')
        lines = []
        remap = {'Designator': 'Ref','Val': 'Val','Package': 'Package','MidX':'PosX','MidY': 'PosY','Rotation': 'Rot', 'Layer': 'Side'}
        
        for line in posreader:
            if line['Ref'] in filter_components:
                continue
        
            lines.append({k1: line[k0] for k1, k0 in remap.items()})
            
        with open('POS.csv','w') as f:
            poswriter = csv.DictWriter(f,delimiter=',',quotechar='"',fieldnames=remap.keys())
            poswriter.writeheader()
            for line in lines:
                poswriter.writerow(line)
            
