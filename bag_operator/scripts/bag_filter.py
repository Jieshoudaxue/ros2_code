#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import argparse
import logging
import rosbag2_py

logging.basicConfig(level=logging.DEBUG)

class BagFilter(object):
    def __init__(self, input_bag, output_dir, start_offset, end_offset, topics):
        self._input_bag = input_bag
        self._output_dir = output_dir
        self._start_offset = start_offset
        self._end_offset = end_offset
        print("{0} {1}".format(self._start_offset, self._end_offset))
        self._topics = topics
        self._reader = rosbag2_py.SequentialReader()
        self._writer = rosbag2_py.SequentialWriter()
        

    def filter_bag(self):
        # 以sqlite3格式打开ros2 bag
        in_storage_options = rosbag2_py._storage.StorageOptions(uri=self._input_bag, storage_id='sqlite3')
        self._reader.open(in_storage_options, rosbag2_py._storage.ConverterOptions('', ''))
        # 设置要读取的topic作为过滤条件，如果没有指定topic，则读取所有topic
        if self._topics:
            storage_filter = rosbag2_py._storage.StorageFilter(topics=self._topics)
            self._reader.set_filter(storage_filter)

        # 以sqlite3格式创建ros2 bag
        out_storage_options = rosbag2_py._storage.StorageOptions(uri=self._output_dir, storage_id='sqlite3')
        self._writer.open(out_storage_options, rosbag2_py._storage.ConverterOptions('', ''))

        # 在 writer 中注册输入bag中所有的topic，为写入做准备
        for topic_metadata in self._reader.get_all_topics_and_types():
            # print(topic_metadata.name)
            topic_info = rosbag2_py._storage.TopicMetadata(
                name = topic_metadata.name,
                type = topic_metadata.type,
                serialization_format='cdr')
            self._writer.create_topic(topic_info)   

        # 获取开始和结束时间：从 datatime.datetime 和 datetime.timedelta 转换为 ns
        in_metadata = rosbag2_py.Info().read_metadata(self._input_bag, 'sqlite3')
        bag_start_time_ns = int(in_metadata.starting_time.timestamp()* 1e9)
        bag_end_time_ns = bag_start_time_ns + int(in_metadata.duration.days * 24 * 60 * 60 * 1e9 + in_metadata.duration.seconds * 1e9 + in_metadata.duration.microseconds * 1e3)
        cut_start_time_ns = int(bag_start_time_ns + self._start_offset * 1e9)
        cut_end_time_ns = int(bag_end_time_ns - self._end_offset * 1e9)

        # 循环读取消息，判断时间范围后，并写入新bag
        while self._reader.has_next():
            (topic_name, data, t) = self._reader.read_next()
            if cut_start_time_ns <= t <= cut_end_time_ns:
                self._writer.write(topic_name, data, t)

        # 获取输出bag的topic列表和meta信息
        out_reader = rosbag2_py.SequentialReader()
        output_bag = os.path.join(self._output_dir, os.path.basename(self._output_dir) + "_0.db3")
        db3_storage_options = rosbag2_py._storage.StorageOptions(uri=output_bag, storage_id='sqlite3')
        out_reader.open(db3_storage_options, rosbag2_py._storage.ConverterOptions('', ''))      
        out_topic_list = [topic_metadata.name for topic_metadata in out_reader.get_all_topics_and_types()]
        out_metadata = rosbag2_py.Info().read_metadata(output_bag, 'sqlite3')

        # 打印摘要信息
        logging.info("------------[%s summary]------------" %self._output_dir)
        logging.info("time span %s [origin %s]" %(out_metadata.duration, in_metadata.duration))
        logging.info("topic list %s" %out_topic_list)

def main():
  parser = argparse.ArgumentParser(description="filter ros2 bag depends time offset and topics")
  parser.add_argument("-i", "--input_bag", type=str, required=True, help="specify input rosbag")
  parser.add_argument("-o", "--output_dir", type=str, required=True, help="specify output rosbag")
  parser.add_argument("-s", "--start_offset_sec", type=int, default=0, help="specify start offset time")
  parser.add_argument("-e", "--end_offset_sec", type=int, default=0, help="specify end offset time")
  parser.add_argument("-t", "--topics", nargs="+", type=str, help="specify topic")
  parser.add_argument('extra_args', nargs='*', help=argparse.SUPPRESS)
  args=parser.parse_args()

  if not os.path.isfile(args.input_bag):
    logging.error("%s is no found!" %args.input_bag)
    return

  filter = BagFilter(args.input_bag, args.output_dir, args.start_offset_sec, args.end_offset_sec, args.topics)
  filter.filter_bag()

if __name__ == "__main__":
    main()