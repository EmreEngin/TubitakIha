import dronekit as dk
from pymavlink import mavutil
import functions as fc
import time
import paramiko
# dronekit-sitl copter --home=40.744400, 30.337950,0,0
# mavproxy --master tcp:localhost:5760 --out localhost:14551 --out localhost:14552
#hata konrtol payini ordaki ortama gore duzenlenecek.
vehicle = dk.connect('udp:localhost:14552', wait_ready=True)

"""sise1 = (40.744177, 30.338282)
sise2 = (40.744180, 30.338255)
yarimdolualan = (40.744563, 30.338123)
tamdolualan = (40.744625, 30.338347)"""
sise1 = fc.konumVer("sise1")
sise2 = fc.konumVer("sise2")
yarimdolualan = fc.konumVer("yarimdolualan")
tamdolualan = fc.konumVer("tamdolualan")
"""
konumlar vehicle.simple_goto ya cevriliyor

"""
sise = fc.konumcevirme(sise1, 4)
sisealcalma = fc.konumcevirme(sise1, 0.75)
siseiki = fc.konumcevirme(sise2, 4)
sise2alcalma = fc.konumcevirme(sise2, 0.75)
yarim = fc.konumcevirme(yarimdolualan, 4)
yarimalcalma = fc.konumcevirme(yarimdolualan, 0.75)
tam = fc.konumcevirme(tamdolualan, 4)
tamalcalma = fc.konumcevirme(tamdolualan, 0.75)

# arm edilip kalis yapiliyor...
fc.arm_and_takeoff(vehicle, 4)
# LOITER mod icin gaz ayarlamasi yapiliyor...
vehicle.channels.overrides['3'] = 1500
# sise1 icin yola cikiliyor...
vehicle.simple_goto(sise, 2)
# konuma gidip gitmedigi kontrol ediliyor..
while not fc.OradaMiyim(vehicle, sise1, 0.25):
    fc.bilgiYaz(vehicle)
    time.sleep(1)
    vehicle.simple_goto(sise, 2)
print("REACH FIRST POINT")
# Alcalma yapiliyor...
vehicle.simple_goto(sisealcalma)
print "DESCENDING... "
time.sleep(5)
vehicle.channels.overrides['3'] = 1500
print "MOD CHANGING..."
vehicle.mode = dk.VehicleMode('LOITER')
print "MOD CHANGED"
fc.cameraac("192.168.1.45", "22", "pi", "tekno5454")
time.sleep(1)
#CONTROL DEGISKENLERI
fc.control(vehicle)
fc.camerakapat("192.168.1.45", "22", "pi", "tekno5454")
print "MOD CHANGING.."
vehicle.mode = dk.VehicleMode('GUIDED')
time.sleep(5)
print " MOD CHANGED"

b = fc.run_download("192.168.1.45", "22", "pi", "tekno5454")

if b == 0:
    vehicle.simple_goto(yarim, 2)
    print "TAKE THE ROAD"
    while not fc.OradaMiyim(vehicle, yarimdolualan, 0.30):
        fc.bilgiYaz(vehicle)
        time.sleep(1)
        vehicle.simple_goto(yarim, 2)

    print "REACH TARGET POINT"

    vehicle.simple_goto(yarimalcalma)
    print "DESCENDING... "
    time.sleep(5)
    vehicle.channels.overrides['3'] = 1500
    print "MOD CHANGING..."
    vehicle.mode = dk.VehicleMode('LOITER')
    print "MOD CHANGED"
    time.sleep(10)
    """print "SERVO IS WORKING..."
    fc.set_servo(vehicle, 5, 1000)"""
    print "MOD CHANGING..."
    vehicle.mode = dk.VehicleMode('GUIDED')
    print " MOD CHANGED"
    time.sleep(1)
    
    vehicle.simple_goto(siseiki, 2)

    print " TAKE THE ROAD "
    while not fc.OradaMiyim(vehicle, sise2, 0.30):
        fc.bilgiYaz(vehicle)
        time.sleep(1)
        vehicle.simple_goto(siseiki, 2)
    print " REACH THIRD POINT 3"

    vehicle.simple_goto(sise2alcalma)
    print "DESCENDING... "
    time.sleep(5)
    vehicle.channels.overrides['3'] = 1500
    print "MOD CHANGING..."
    vehicle.mode = dk.VehicleMode('LOITER')
    print "MOD CHANGED"
    fc.cameraac("192.168.1.45", "22", "pi", "tekno5454")
    time.sleep(1)
    fc.control(vehicle)
    fc.camerakapat("192.168.1.45", "22", "pi", "tekno5454")
    time.sleep(1)
    print "MOD CHANGING..."
    vehicle.mode = dk.VehicleMode('GUIDED')
    print "MOD CHANGED"

    vehicle.simple_goto(tam, 2)
    while not fc.OradaMiyim(vehicle, tamdolualan, 0.30):
        fc.bilgiYaz(vehicle)
        time.sleep(1)
        vehicle.simple_goto(tam, 2)
    print " REACH FOURTH POINT"
    vehicle.simple_goto(tamalcalma)
    print "DESCENDING... "
    time.sleep(5) 
    vehicle.channels.overrides['3'] = 1500
    print "MOD CHANGING..."
    vehicle.mode = dk.VehicleMode('LOITER')
    print "MOD CHANGED"   
    time.sleep(1)
    """print "SERVO IS WORKING"
    fc.set_servo(vehicle,5 ,1000)"""
    print " MOD CHANGING"
    vehicle.mode = dk.VehicleMode('RTL')
    print " MOD CHANGED"

else:
    vehicle.simple_goto(tam, 2)
    while not fc.OradaMiyim(vehicle, tamdolualan, 0.30):
        fc.bilgiYaz(vehicle)
        time.sleep(1)
        vehicle.simple_goto(tam, 2)
    print " REACH FOURTH POINT"

    vehicle.simple_goto(tamalcalma)
    print "DESCENDING... "
    time.sleep(5)
    vehicle.channels.overrides['3'] = 1500
    print "MOD CHANGING..."
    vehicle.mode = dk.VehicleMode('LOITER')
    print "MOD CHANGED"
    time.sleep(1)   
    """print"SERVO IS WORKING"
    fc.set_servo(vehicle,5,1500)"""
    print "MOD CHANGING..."
    vehicle.mode = dk.VehicleMode('GUIDED')
    print "MOD CHANGED"

    vehicle.simple_goto(siseiki, 2)

    print " TAKE THE ROAD "
    while not fc.OradaMiyim(vehicle, sise2, 0.30):
        fc.bilgiYaz(vehicle)
        time.sleep(1)
        vehicle.simple_goto(siseiki, 2)
    print " REACH THIRD POINT 3"

    vehicle.simple_goto(sise2alcalma)
    print "DESCENDING... "
    time.sleep(5)
    vehicle.channels.overrides['3'] = 1500
    print "MOD CHANGING..."
    vehicle.mode = dk.VehicleMode('LOITER')
    print "MOD CHANGED"
    fc.cameraac("192.168.1.45", "22", "pi", "tekno5454")
    fc.control(vehicle)
    fc.camerakapat("192.168.1.45", "22", "pi", "tekno5454")
    time.sleep(1)
    print "MOD CHANGING..."
    vehicle.mode = dk.VehicleMode('GUIDED')
    print "MOD CHANGED"

    vehicle.simple_goto(yarim, 2)
    print "TAKE THE ROAD"
    while not fc.OradaMiyim(vehicle, yarimdolualan, 0.30):
        fc.bilgiYaz(vehicle)
        time.sleep(1)
        vehicle.simple_goto(yarim, 2)

    print "REACH TARGET POINT"

    vehicle.simple_goto(yarimalcalma)
    print "DESCENDING... "
    time.sleep(5)
    vehicle.channels.overrides['3'] = 1500
    print "MOD CHANGING..."
    vehicle.mode = dk.VehicleMode('LOITER')
    print "MOD CHANGED"
    time.sleep(1)
    """print "SERVO IS WORKING..."
    fc.set_servo(vehicle, 5, 1000)"""
    print "MOD CHANGING..."
    vehicle.mode = dk.VehicleMode('RTL')
    print " MOD CHANGED"
vehicle.close()
