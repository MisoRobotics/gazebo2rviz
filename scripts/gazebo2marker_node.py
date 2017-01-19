#!/usr/bin/env python

import argparse
from gazebo_msgs.msg import ModelStates
from gazebo2rviz import *
import pysdf
import rospy
import tf
from visualization_msgs.msg import Marker


class Gazebo2MarkerNode:
  def __init__(self, update_period, marker_topic,
               use_collision=False, submodels_to_be_ignored=None):
    rospy.loginfo('Initializing %s' % self.__class__.__name__)

    if submodels_to_be_ignored is not None:
      rospy.loginfo('Ignoring submodels of: ' + str(submodelsToBeIgnored))

    self.update_period = update_period
    self.use_collision = use_collision
    self.marker_pub = rospy.Publisher(marker_topic, Marker, queue_size=1)
    self.model_states_sub = rospy.Subscriber('gazebo/model_states', ModelStates,
                                             self.on_model_states_msg)
    self.model_cache = {}
    self.world = None

  def publish_link_marker(link, full_linkname, **kwargs):
    full_linkinstancename = full_linkname
    if 'model_name' in kwargs and 'instance_name' in kwargs:
      full_linkinstancename = -++full_linkinstancename.replace(kwargs['model_name'],
                                                          kwargs['instance_name'], 1)
    marker_msgs = link2marker_msg(link, full_linkinstancename, use_collision,
                                rospy.Duration(2 * updatePeriod))
    if len(marker_msgs) > 0:
      for marker_msg in marker_msgs:
        markerPub.publish(marker_msg)

  def on_model_states_msg(self, model_states_msg):
    time_since_last_update = rospy.get_rostime() - self.last_update_time
    if time_since_last_update.to_sec() < self.update_period:
      return
    last_update_time = rospy.Time.now()

    for model_idx, modelinstance_name in enumerate(model_states_msg.name):
      # print(model_idx, modelinstance_name)
      model_name = pysdf.name2modelname(modelinstance_name)
      # print('model_name:', model_name)
      if not model_name in model_cache:
        sdf = pysdf.SDF(model=model_name)
        self.model_cache[model_name] = sdf.world.models[0] \
                                       if len(sdf.world.models) >= 1 else None
        if self.model_cache[model_name]:
          print('Loaded model: %s' % self.model_cache[model_name].name)
        else:
          print('Unable to load model: %s' % model_name)
          model = self.model_cache[model_name]
      if not model: # Not an SDF model
        continue
    # print('model:', model)
    model.for_all_links(self.publish_link_marker,
                        model_name=model_name,
                        instance_name=modelinstance_name)


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('-f', '--freq', type=float, default=2,
                      help='Frequency Markers are published (default: 2 Hz)')
  parser.add_argument('-c', '--collision', action='store_true',
                      help='Publish collision instead of visual elements')
  args = parser.parse_args(rospy.myargv()[1:])

  rospy.init_node('gazebo2marker')

  submodels = rospy.get_param('~ignore_submodels_of', '').split(';')
  update_period = 1. / args.freq
  use_collision = args.collision
  marker_topic = '/visualization_marker'

  node = Gazebo2MarkerNode(update_period, marker_topic,
                           use_collision=use_collision,
                           submodels_to_be_ignored=submodels)
  rospy.spin()
