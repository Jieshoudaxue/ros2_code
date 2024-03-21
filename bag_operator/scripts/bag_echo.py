#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import argparse
import logging
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

logging.basicConfig(level=logging.DEBUG)

class BagEcho(object):
    def __init__(self, bag, topics):
        self._bag = bag
        self._topics = topics
        self._reader = rosbag2_py.SequentialReader()

    def echo_bag(self):
        # 以sqlite3格式打开ros2 bag
        storage_options = rosbag2_py._storage.StorageOptions(uri=self._bag, storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self._reader.open(storage_options, converter_options)
        # 设置要读取的topic作为过滤条件，如果没有指定topic，则读取所有topic
        if self._topics:
            storage_filter = rosbag2_py._storage.StorageFilter(topics=self._topics)
            self._reader.set_filter(storage_filter)
        # 获取所有话题及其类型，并使用map存储
        topic_name2type = {topic_metadata.name: topic_metadata.type for topic_metadata in self._reader.get_all_topics_and_types()}

        # 循环读取消息，并进行个数统计
        msg_cnts = {}        
        while self._reader.has_next():
            (topic_name, data, t) = self._reader.read_next()

            # 统计每个话题的消息数
            if topic_name not in msg_cnts:  
                msg_cnts[topic_name] = 1
            else:
                msg_cnts[topic_name] += 1            

            # 反序列化消息
            message_type = get_message(topic_name2type[topic_name])
            msg = deserialize_message(data, message_type)

            # 打印消息内容
            print("----------[%s : %s]---------" % (topic_name, topic_name2type[topic_name]))
            print(msg)            

        # 打印摘要信息
        logging.info("----------[rosbag summary]---------")
        for topic_name in msg_cnts:  
            logging.info("topic %s msg cnt is %d" %(topic_name, msg_cnts[topic_name]))

def main():
  parser = argparse.ArgumentParser(description="echo ros2 bag topic")
  parser.add_argument("-b", "--bag", type=str, required=True, help="specify rosbag")
  parser.add_argument("-t", "--topics", nargs="+", type=str, help="specify topic")
  # extra_args可以让-t指定多个topic，并使用argparse.SUPPRESS隐藏extra_args参数
  parser.add_argument('extra_args', nargs='*', help=argparse.SUPPRESS)
  args=parser.parse_args()

  if not os.path.isfile(args.bag):
      logging.error("%s is not found!" %args.bag)
      return

  echo = BagEcho(args.bag, args.topics)
  echo.echo_bag()

if __name__ == "__main__":
    main()