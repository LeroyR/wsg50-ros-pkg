#!/usr/bin/env python
import rospy
import requests
from enum import Enum, IntEnum
from wsg_50_driver.srv import *

class WSG50MotionConf(IntEnum):
    PWT = 253 #Part Width Tolerance
    CT = 260 #Clamping Travel

def send_response(_param, _param_value):
    response = requests.post(
    u'http://'+_ip+'/graspconf.json',

    headers={u'Content-Length': u'108', u'Accept-Language': u'en-GB,en;q=0.5', u'Accept-Encoding': u'gz_ip, deflate', u'Connection': u'keep-alive', u'Accept': u'*/*', u'User-Agent': u'rosvita', u'Host': _ip, u'Referer': u'http://'+_ip+'/graspconf.html', u'X-Requested-With': u'XMLHttpRequest', u'Content-Type': u'application/x-www-form-urlencoded; charset=UTF-8'},

    data=str(int(_param))+'='+str(_param_value)+'&sender_id=342&event_type=0',
    )

def pwt_service_handle(req):
    send_response(WSG50MotionConf.PWT, req.value)
    return SetWsgParamResponse()

def ct_service_handle(req):
    send_response(WSG50MotionConf.CT, req.value)
    return SetWsgParamResponse()

if __name__ == '__main__':
    rospy.init_node('wsg_50_web_services')
    _ip = rospy.get_param("~_ip", '192.168.50.33')

    #declare services
    rospy.Service('~set_part_width_tolerance', SetWsgParam, pwt_service_handle)
    rospy.Service('~set_clamping_travel', SetWsgParam, ct_service_handle)

    rospy.spin()