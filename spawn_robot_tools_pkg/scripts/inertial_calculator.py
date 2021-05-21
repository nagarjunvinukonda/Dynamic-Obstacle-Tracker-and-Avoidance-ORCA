#!/usr/bin/env python
import math


class InertialCalculator(object):

    def __init__(self):
        print "InertialCaluclator Initialised..."

    def start_ask_loop(self):

        selection = "START"

        while selection != "Q":
            print "#############################"
            print "Select Geometry to Calculate:"
            print "[1]Box width(w)*depth(d)*height(h)"
            print "[2]Sphere radius(r)"
            print "[3]Cylinder radius(r)*height(h)"
            print "[Q]END program"
            selection = raw_input(">>")
            self.select_action(selection)

        print "InertialCaluclator Quit...Thank you"

    def select_action(self, selection):
        if selection == "1":
            mass = float(raw_input("mass>>"))
            width = float(raw_input("width>>"))
            depth = float(raw_input("depth>>"))
            height = float(raw_input("height>>"))
            self.calculate_box_inertia(m=mass, w=width, d=depth, h=height)
        elif selection == "2":
            mass = float(raw_input("mass>>"))
            radius = float(raw_input("radius>>"))
            self.calculate_sphere_inertia(m=mass, r=radius)
        elif selection == "3":
            mass = float(raw_input("mass>>"))
            radius = float(raw_input("radius>>"))
            height = float(raw_input("height>>"))
            self.calculate_cylinder_inertia(m=mass, r=radius, h=height)
        elif selection == "Q":
            print "Selected Quit"
        else:
            print "Usage: Select one of the give options"


    def calculate_box_inertia(self, m, w, d, h):
        Iw = (m/12.0)*(pow(d,2)+pow(h,2))
        Id = (m / 12.0) * (pow(w, 2) + pow(h, 2))
        Ih = (m / 12.0) * (pow(w, 2) + pow(d, 2))
        print "BOX w*d*h, Iw = "+str(Iw)+",Id = "+str(Id)+",Ih = "+str(Ih)

    def calculate_sphere_inertia(self, m, r):
        I = (2*m*pow(r,2))/5.0
        print "SPHERE Ix,y,z = "+str(I)

    def calculate_cylinder_inertia(self, m, r, h):
        Ix = (m/12.0)*(3*pow(r,2)+pow(h,2))
        Iy = Ix
        Iz = (m*pow(r,2))/2.0
        print "Cylinder Ix,y = "+str(Ix)+",Iz = "+str(Iz)

if __name__ == "__main__":
    inertial_object = InertialCalculator()
    inertial_object.start_ask_loop()