import odrive
from odrive.enums import *
odrv0=odrive.find_any()

#run boith wheels forward at .02
#odrv0.axis0.requested_state=AxisState.FULL_CALIBRATION_SEQUENCE
#odrv0.axis1.requested_state=AxisState.FULL_CALIBRATION_SEQUENCE

#set control and velocity
#odrv0.axis1.encoder.pos_estimate_counts#this is encoder counts decimal
#odrv0.axis1.encoder.shadow_count #this is total encoder counts

odrv0.axis0.controller.input_vel=.2
odrv0.axis1.controller.input_vel=-.2


odrv0.axis0.encoder.set_linear_count(0) #this reset encoder
odrv0.axis1.encoder.set_linear_count(0) #this reset encoder

requestedistance=1 #in m
wheeldiameter=.17
requiredcprcount = (requestedistance/(3.14*wheeldiameter))*3200

odrv0.axis0.requested_state=AxisState.CLOSED_LOOP_CONTROL
odrv0.axis1.requested_state=AxisState.CLOSED_LOOP_CONTROL


Lsc=odrv0.axis0.encoder.shadow_count 
Rsc=odrv0.axis0.encoder.shadow_count 

while Lsc < requiredcprcount and Rsc > -requiredcprcount:
    #pec=odrv0.axis0.encoder.pos_estimate_counts #this might be total encoder counts
    Lsc=odrv0.axis0.encoder.shadow_count #this might be total encoder counts
    Rsc=odrv0.axis1.encoder.shadow_count #this might be total encoder counts

    print(Lsc,Rsc)
    j=0
    while j<1000:
        j=j+1

odrv0.axis0.requested_state=AxisState.IDLE
odrv0.axis1.requested_state=AxisState.IDLE



