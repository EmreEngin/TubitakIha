import sys
from dronekit import connect

vehicle = connect("udp:localhost:14552", wait_ready= True)

def konumKaydet(nesne):                                                                       
    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon
    if(nesne == "sag"):
        with open("sag.txt", "w") as file:
            file.write(str(lat) + "\n" + str(lon) )
    elif(nesne == "sol"):
        with open("sol.txt", "w") as file:
            file.write(str(lat) + "\n" + str(lon) )

    elif(nesne == "sise1"):
        with open("sise1.txt", "w") as file:
            file.write(str(lat) + "\n" + str(lon) )
    elif(nesne == "sise2"):
        with open("sise2.txt", "w") as file:
            file.write(str(lat) + "\n" + str(lon) )
    elif(nesne == "yarimdolualan"):
        with open("yarimdolualan.txt", "w") as file:
            file.write(str(lat) + "\n" + str(lon) )

    elif (nesne =="tamdolualan"):
        with open("tamdolualan.txt", "w") as file:
            file.write(str(lat) + "\n" + str(lon) )


        
if __name__ == "__main__":
   konumKaydet(sys.argv[1])