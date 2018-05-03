#!/usr/bin/env python

import requests
import json
import rospy

from std_msgs.msg import String

transaction_count_publisher = rospy.Publisher('transactionCountResult', String, queue_size=10)

def transactionCountRequestCallback(message):
    rospy.loginfo('Transaction count request received. Passing it to Infura.')

    infura_url = 'https://ropsten.infura.io/k1qUYiOPy281bLyAQmDA'

    json_request = '{"jsonrpc": "2.0", "id": 1, "method": "eth_getTransactionCount","params": ["' + message.data + '", "latest"]}'   
    response = requests.post(infura_url, data=json_request)

    transaction_count = int(json.loads(response.text)["result"], 0)

    rospy.loginfo('Transaction count: %s', transaction_count)

    transaction_count_publisher.publish(str(transaction_count))

def listener():
    rospy.init_node('listener', anonymous = True)

    rospy.Subscriber('transactionCountRequest', String, transactionCountRequestCallback)

    rospy.spin()

if __name__ == '__main__':
    listener()