# Copyright: (C) 2019 iCub Tech Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

# This script generates the ball shape model
# composed of initial estimates of its radius

import argparse
import math

parser = argparse.ArgumentParser()
parser.add_argument("-r", "--radius", type=float,
                    help="ball radius in millimeters (default: 30)", default=30)
parser.add_argument("-p", "--percentage", type=float,
                    help="percentage in [0, 100] for inner and outer radii (default: 20)", default=20)
parser.add_argument("-n", "--points", type=int,
                    help="number of points generated (default: 50)", default=50)
parser.add_argument("-f", "--file", type=str,
                    help="name of the generated file (default: shape_model.csv)", default='shape_model.csv')
args = parser.parse_args()

print('using:')
print('radius     = {} [mm]'.format(args.radius))
print('percentage = {} [%]'.format(args.percentage))
print('points     = {}'.format(args.points))
print('file       = "{}"'.format(args.file))

R_i = (1.0 - (args.percentage / 100.0)) * args.radius
R_o = (1.0 + (args.percentage / 100.0)) * args.radius

x = []
y = []
z = []
t = 0.0
t_delta = (2.0 * math.pi) / args.points
for i in range(args.points):
    x.append(0.0)
    y.append(math.sin(t))
    z.append(math.cos(t))
    t += t_delta

fout = open(args.file, "w")
for i in range(args.points):
    fout.write('{0:.3f}\n'.format(x[i]))
for i in range(args.points):
    fout.write('{0:.3f}\n'.format(x[i]))
for i in range(args.points):
    fout.write('{0:.3f}\n'.format(R_i * y[i]))
for i in range(args.points):
    fout.write('{0:.3f}\n'.format(R_o * y[i]))
for i in range(args.points):
    fout.write('{0:.3f}\n'.format(R_i * z[i]))
for i in range(args.points):
    fout.write('{0:.3f}\n'.format(R_o * z[i]))
fout.close()
