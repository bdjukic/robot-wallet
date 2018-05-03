#!/usr/bin/env python

import requests
import json
import rospy

from std_msgs.msg import String

def smartContractRequestCallback(message):
    infura_url = 'https://ropsten.infura.io/k1qUYiOPy281bLyAQmDA'

    json_request = '{"jsonrpc": "2.0", "id": 3, "method": "eth_sendRawTransaction","params": ["' + message.data + '"]}'   
    response = requests.post(infura_url, data=json_request)

    rospy.loginfo('Response: %s', response.text)

def listener():
    rospy.init_node('smartContractRequestListener', anonymous = True)

    rospy.Subscriber('smartContractRequest', String, smartContractRequestCallback)
    
    rospy.spin()

if __name__ == '__main__':
    listener()