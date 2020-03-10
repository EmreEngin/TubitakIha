import dronekit as dk
import os


def konumVer(nesne):
    if nesne == "sag":
        with open("sag.txt", "r") as a:
            dizi = a.readlines()
            dizi = [x.strip() for x in dizi]
            latsag = float(dizi[0])
            lonsag = float(dizi[1])
            sagdirek = (latsag, lonsag)
            return sagdirek

    elif nesne == "sol":
        with open("sol.txt", "r") as o:
            dizi =o.readlines()
            dizi = [x.strip() for x in dizi]
            latsol = float(dizi[0])
            lonsol = float(dizi[1])
            soldirek = (latsol, lonsol)
            return soldirek
    elif nesne == "sise1":
        with open("sise1.txt", "r") as bir:
            dizi =bir.readlines()
            dizi = [x.strip() for x in dizi]
            latsise1 = float(dizi[0])
            lonsise1 = float(dizi[1])
            sise1 = (latsise1, lonsise1)
            return sise1

    elif nesne == "sise2":
        with open("sise2.txt", "r") as iki:
            dizi =iki.readlines()
            dizi = [x.strip() for x in dizi]
            latsise2 = float(dizi[0])
            lonsise2 = float(dizi[1])
            sise2 = (latsise2, lonsise2)
            return sise2
    elif nesne == "yarimdolualan":
        with open("yarimdolualan.txt", "r") as y:
            dizi =y.readlines()
            dizi = [x.strip() for x in dizi]
            laty = float(dizi[0])
            lony = float(dizi[1])
            yarimdolualan = (laty, lony)
            return yarimdolualan
    elif nesne == "tamdolualan":
        with open("tamdolualan.txt", "r") as t:
            dizi =t.readlines()
            dizi = [x.strip() for x in dizi]
            latt = float(dizi[0])
            lont = float(dizi[1])
            tamdolualan = (latt, lont)
            return tamdolualan



