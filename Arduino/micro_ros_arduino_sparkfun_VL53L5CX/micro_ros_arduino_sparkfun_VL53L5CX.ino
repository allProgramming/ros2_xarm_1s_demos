#include <micro_ros_arduino.h>
#include <micro_ros_utilities/string_utilities.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/point_cloud2.h>
#include <std_msgs/msg/int32.h>

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

#define RESOLUTION        8 // in one dimension (8*8=64)
#define FIELDS_PER_PT     3 // x, y, z
#define BYTES_PER_FIELD   4 // float 32
#define HALF_PI           (PI / 2)
#define FOV               HALF_PI
#define HALF_FOV          (FOV / 2.0)
#define FOV_PER_RES       (FOV / RESOLUTION)
#define TIMER_TIMEOUT     100

#define MM_PER_METER      1000.0
#define NANOS_PER_SEC     1000000000

#define LED_PIN           13

void error_loop() {
  while(1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

union
{
  float number;
  uint8_t bytes[4];
} field;

rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

SparkFun_VL53L5CX imager;
VL53L5CX_ResultsData measurements; // Result data class structure, 1356 byes of RAM
int depth;
sensor_msgs__msg__PointCloud2 msg;
uint8_t msg_point_cloud[RESOLUTION * RESOLUTION * FIELDS_PER_PT * BYTES_PER_FIELD];
sensor_msgs__msg__PointField fields[3];
rcl_publisher_t publisher;

int time_sync_count = 0;
int64_t total_nanos;

void timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer == NULL)
    return;

  if (time_sync_count % 600 == 0)
    rmw_uros_sync_session(1000); // Synchronize time with the agent
  time_sync_count++;

  // Poll sensor for new data
  if (!imager.isDataReady())
    return;

  if (!imager.getRangingData(&measurements))
    return;

  uint8_t* point_cloud_write_ptr = msg_point_cloud;
  for(int w = 0; w < RESOLUTION; w++)
  {
    for(int h = 0; h < RESOLUTION; h++)
    {
      // TO-DO: float-NaN, instead of 0, for invalid data
      depth = max(measurements.distance_mm[w + h * RESOLUTION], (int16_t)0);

      // x field
      field.number = (cos(w * FOV_PER_RES - HALF_FOV - HALF_PI) * depth) / MM_PER_METER;
      memcpy(point_cloud_write_ptr, field.bytes, sizeof(field.bytes));
      point_cloud_write_ptr += sizeof(field.bytes);

      // y field
      field.number = (sin(h * FOV_PER_RES - HALF_FOV) * depth) / MM_PER_METER;
      memcpy(point_cloud_write_ptr, field.bytes, sizeof(field.bytes));
      point_cloud_write_ptr += sizeof(field.bytes);

      // z field
      field.number = depth / MM_PER_METER;
      memcpy(point_cloud_write_ptr, field.bytes, sizeof(field.bytes));
      point_cloud_write_ptr += sizeof(field.bytes);
    }
  }

  total_nanos = rmw_uros_epoch_nanos();
  msg.header.stamp.sec = total_nanos / NANOS_PER_SEC;
  msg.header.stamp.nanosec = total_nanos % NANOS_PER_SEC;

  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
}

void setup()
{
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Wire.begin(); // 100kHz I2C
  Wire.setClock(400000); // Sensor has max I2C freq of 400kHz 

  imager.begin();
  imager.setResolution(RESOLUTION * RESOLUTION); // Enable all 64 pads
  imager.startRanging();

  // Static point cloud 2 message fields
  msg.header.frame_id = micro_ros_string_utilities_set(msg.header.frame_id, "depth");
  msg.height = msg.width = RESOLUTION;
  fields[0].name = micro_ros_string_utilities_set(fields[0].name, "x");
  fields[0].offset = BYTES_PER_FIELD * 0;
  fields[1].name = micro_ros_string_utilities_set(fields[1].name, "y");
  fields[1].offset = BYTES_PER_FIELD * 1;
  fields[2].name = micro_ros_string_utilities_set(fields[2].name, "z");
  fields[2].offset = BYTES_PER_FIELD * 2;
  fields[0].datatype = fields[1].datatype = fields[2].datatype = sensor_msgs__msg__PointField__FLOAT32;
  fields[0].count = fields[1].count = fields[2].count = 1;
  msg.fields.size = FIELDS_PER_PT; // x,y,z
  msg.fields.data = fields;
  msg.is_bigendian = false;
  msg.point_step = FIELDS_PER_PT * BYTES_PER_FIELD;
  msg.row_step = msg.point_step * msg.width;
  msg.data.size = msg.row_step * msg.height;
  msg.data.data = msg_point_cloud;
  msg.is_dense = false;

  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "sparkfun_vl53l5cx_node", "", &support));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, PointCloud2),
    "point_cloud"));

  // Create timer
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(TIMER_TIMEOUT),
    timer_callback));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop()
{
  delay(5);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(TIMER_TIMEOUT)));
}
