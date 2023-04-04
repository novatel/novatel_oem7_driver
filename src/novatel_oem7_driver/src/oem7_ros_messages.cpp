////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020 NovAtel Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////
//
// The names of messages are ROS messages, not Oem7 messages. These may be identical, but not necessarily.
// More than one Oem7 message may be used to generate the same ROS message
//

#include "novatel_oem7_driver/oem7_ros_messages.hpp"

#include "novatel_oem7_driver/oem7_message_ids.h"
#include "novatel_oem7_driver/oem7_messages.h"
#include "novatel_oem7_driver/oem7_message_util.hpp"


#include "novatel_oem7_msgs/msg/heading2.hpp"
#include "novatel_oem7_msgs/msg/bestpos.hpp"
#include "novatel_oem7_msgs/msg/bestvel.hpp"
#include "novatel_oem7_msgs/msg/bestutm.hpp"
#include "novatel_oem7_msgs/msg/bestgnsspos.hpp"
#include "novatel_oem7_msgs/msg/ppppos.hpp"
#include "novatel_oem7_msgs/msg/inspva.hpp"
#include "novatel_oem7_msgs/msg/inspvax.hpp"
#include "novatel_oem7_msgs/msg/insconfig.hpp"
#include "novatel_oem7_msgs/msg/insstdev.hpp"
#include "novatel_oem7_msgs/msg/corrimu.hpp"
#include "novatel_oem7_msgs/msg/rxstatus.hpp"
#include "novatel_oem7_msgs/msg/terrastarinfo.hpp"
#include "novatel_oem7_msgs/msg/terrastarstatus.hpp"
#include "novatel_oem7_msgs/msg/time.hpp"



#define arr_size(_arr_) (sizeof(_arr_) / sizeof(_arr_[0]))

namespace novatel_oem7_driver
{

/*
 * Populates Oem7header from raw message
 */
void
SetOem7Header(
    const Oem7RawMessageIf::ConstPtr& msg, ///< in: raw message
    const std::string& name, ///< message name
    novatel_oem7_msgs::msg::Oem7Header::Type& oem7_hdr ///< header to populate
    )
{
  getOem7Header(msg, oem7_hdr);
  oem7_hdr.message_name = name;
}

/*
 * Populates Oem7header from a 'short' raw message
 */
void
SetOem7ShortHeader(
    const Oem7RawMessageIf::ConstPtr& msg, ///< in: short raw message
    const std::string& name, ///< message name
    novatel_oem7_msgs::msg::Oem7Header::Type& oem7_hdr ///< header to populate
    )
{
  getOem7ShortHeader(msg, oem7_hdr);
  oem7_hdr.message_name = name;
}

// Template specializations from specific messages

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::HEADING2>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::HEADING2>& heading2)
{
  assert(msg->getMessageId() == HEADING2_OEM7_MSGID);

  const HEADING2Mem* mem = reinterpret_cast<const HEADING2Mem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  heading2.reset(new novatel_oem7_msgs::msg::HEADING2);

  heading2->sol_status.status     = mem->sol_status;
  heading2->pos_type.type         = mem->pos_type;
  heading2->length                = mem->length;
  heading2->heading               = mem->heading;
  heading2->pitch                 = mem->pitch;
  heading2->reserved              = mem->reserved;
  heading2->heading_stdev         = mem->heading_stdev;
  heading2->pitch_stdev           = mem->pitch_stdev;
  std::copy(std::begin(mem->rover_stn_id),  std::end(mem->rover_stn_id),  std::begin(heading2->rover_stn_id));
  std::copy(std::begin(mem->master_stn_id), std::end(mem->master_stn_id), std::begin(heading2->master_stn_id));
  heading2->num_sv_tracked          = mem->num_sv_tracked;
  heading2->num_sv_in_sol           = mem->num_sv_in_sol;
  heading2->num_sv_obs              = mem->num_sv_obs;
  heading2->num_sv_multi            = mem->num_sv_multi;
  heading2->sol_source.source       = mem->sol_source;
  heading2->ext_sol_status.status   = mem->ext_sol_status;
  heading2->galileo_beidou_sig_mask = mem->galileo_beidou_sig_mask;
  heading2->gps_glonass_sig_mask    = mem->gps_glonass_sig_mask;

  static const std::string name = "HEADING2";
  SetOem7Header(msg, name, heading2->nov_header);
}

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::BESTPOS>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::BESTPOS>& bestpos)
{
  assert(msg->getMessageId() == BESTPOS_OEM7_MSGID);

  const BESTPOSMem* bp = reinterpret_cast<const BESTPOSMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  bestpos.reset(new novatel_oem7_msgs::msg::BESTPOS);

  bestpos->sol_status.status      = bp->sol_stat;
  bestpos->pos_type.type          = bp->pos_type;
  bestpos->lat                    = bp->lat;
  bestpos->lon                    = bp->lon;
  bestpos->hgt                    = bp->hgt;
  bestpos->undulation             = bp->undulation;
  bestpos->datum_id               = bp->datum_id;
  bestpos->lat_stdev              = bp->lat_stdev;
  bestpos->lon_stdev              = bp->lon_stdev;
  bestpos->hgt_stdev              = bp->hgt_stdev;
  std::copy(std::begin(bp->stn_id), std::end(bp->stn_id), std::begin(bestpos->stn_id));
  bestpos->diff_age               = bp->diff_age;
  bestpos->sol_age                = bp->sol_age;
  bestpos->num_svs                = bp->num_svs;
  bestpos->num_sol_svs            = bp->num_sol_svs;
  bestpos->num_sol_l1_svs         = bp->num_sol_l1_svs;
  bestpos->num_sol_multi_svs      = bp->num_sol_multi_svs;
  bestpos->reserved               = bp->reserved;
  bestpos->ext_sol_stat.status    = bp->ext_sol_stat;
  bestpos->galileo_beidou_sig_mask= bp->galileo_beidou_sig_mask;
  bestpos->gps_glonass_sig_mask   = bp->gps_glonass_sig_mask;

  static const std::string name = "BESTPOS";
  SetOem7Header(msg, name, bestpos->nov_header);
}

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::BESTVEL>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::BESTVEL>& bestvel)
{
  assert(msg->getMessageId() == BESTVEL_OEM7_MSGID);

  const BESTVELMem* bv = reinterpret_cast<const BESTVELMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  bestvel.reset(new novatel_oem7_msgs::msg::BESTVEL);

  bestvel->sol_status.status = bv->sol_stat;
  bestvel->vel_type.type     = bv->vel_type;
  bestvel->latency           = bv->latency;
  bestvel->diff_age          = bv->diff_age;
  bestvel->hor_speed         = bv->hor_speed;
  bestvel->trk_gnd           = bv->track_gnd;
  bestvel->ver_speed         = bv->ver_speed;
  bestvel->reserved          = bv->reserved;

  static const std::string name = "BESTVEL";
  SetOem7Header(msg, name, bestvel->nov_header);
}

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::BESTUTM>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::BESTUTM>& bestutm)
{
    assert(msg->getMessageId() == BESTUTM_OEM7_MSGID);

    const BESTUTMMem* mem = reinterpret_cast<const BESTUTMMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
    bestutm.reset(new novatel_oem7_msgs::msg::BESTUTM);

    bestutm->pos_type.type          = mem->pos_type;;
    bestutm->lon_zone_number        = mem->lon_zone_number;
    bestutm->lat_zone_letter        = mem->lat_zone_letter;
    bestutm->northing               = mem->northing;
    bestutm->easting                = mem->easting;
    bestutm->height                 = mem->height;
    bestutm->undulation             = mem->undulation;
    bestutm->datum_id               = mem->datum_id;
    bestutm->northing_stddev        = mem->northing_stddev;
    bestutm->easting_stddev         = mem->easting_stddev;
    bestutm->height_stddev          = mem->height_stddev;
    std::copy(std::begin(mem->stn_id), std::end(mem->stn_id), std::begin(bestutm->stn_id));
    bestutm->diff_age               = mem->diff_age;
    bestutm->sol_age                = mem->sol_age;
    bestutm->num_svs                = mem->num_svs;
    bestutm->num_sol_svs            = mem->num_sol_svs;
    bestutm->num_sol_ggl1_svs       = mem->num_sol_ggl1_svs;
    bestutm->num_sol_multi_svs      = mem->num_sol_multi_svs;
    bestutm->reserved               = mem->reserved;
    bestutm->ext_sol_stat.status    = mem->ext_sol_stat;
    bestutm->galileo_beidou_sig_mask= mem->galileo_beidou_sig_mask;
    bestutm->gps_glonass_sig_mask   = mem->gps_glonass_sig_mask;

    static const std::string name = "BESTUTM";
    SetOem7Header(msg, name, bestutm->nov_header);
  }

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::BESTGNSSPOS>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::BESTGNSSPOS>& bestgnsspos)
{
  assert(msg->getMessageId() == BESTGNSSPOS_OEM7_MSGID);

  const BESTGNSSPOSMem* bgp = reinterpret_cast<const BESTGNSSPOSMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  bestgnsspos.reset(new novatel_oem7_msgs::msg::BESTGNSSPOS);

  bestgnsspos->sol_status.status      = bgp->sol_stat;
  bestgnsspos->pos_type.type          = bgp->pos_type;
  bestgnsspos->lat                    = bgp->lat;
  bestgnsspos->lon                    = bgp->lon;
  bestgnsspos->hgt                    = bgp->hgt;
  bestgnsspos->undulation             = bgp->undulation;
  bestgnsspos->datum_id               = bgp->datum_id;
  bestgnsspos->lat_stdev              = bgp->lat_stdev;
  bestgnsspos->lon_stdev              = bgp->lon_stdev;
  bestgnsspos->hgt_stdev              = bgp->hgt_stdev;
  bestgnsspos->stn_id.assign(           bgp->stn_id, arr_size(bgp->stn_id));
  bestgnsspos->diff_age               = bgp->diff_age;
  bestgnsspos->sol_age                = bgp->sol_age;
  bestgnsspos->num_svs                = bgp->num_svs;
  bestgnsspos->num_sol_svs            = bgp->num_sol_svs;
  bestgnsspos->num_sol_l1_svs         = bgp->num_sol_l1_svs;
  bestgnsspos->num_sol_multi_svs      = bgp->num_sol_multi_svs;
  bestgnsspos->reserved               = bgp->reserved;
  bestgnsspos->ext_sol_stat.status    = bgp->ext_sol_stat;
  bestgnsspos->galileo_beidou_sig_mask= bgp->galileo_beidou_sig_mask;
  bestgnsspos->gps_glonass_sig_mask   = bgp->gps_glonass_sig_mask;

  static const std::string name = "BESTGNSSPOS";
  SetOem7Header(msg, name, bestgnsspos->nov_header);
}

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::INSPVA>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::INSPVA>& pva)
{
  assert(msg->getMessageId() == INSPVAS_OEM7_MSGID);

  const INSPVASmem* pvamem = reinterpret_cast<const INSPVASmem*>(msg->getMessageData(OEM7_BINARY_MSG_SHORT_HDR_LEN));
  pva.reset(new novatel_oem7_msgs::msg::INSPVA);

  pva->latitude        =     pvamem->latitude;
  pva->longitude       =     pvamem->longitude;
  pva->height          =     pvamem->height;
  pva->north_velocity  =     pvamem->north_velocity;
  pva->east_velocity   =     pvamem->east_velocity;
  pva->up_velocity     =     pvamem->up_velocity;
  pva->roll            =     pvamem->roll;
  pva->pitch           =     pvamem->pitch;
  pva->azimuth         =     pvamem->azimuth;
  pva->status.status   =     pvamem->status;

  static const std::string name = "INSPVA";
  SetOem7ShortHeader(msg, name, pva->nov_header);
}


template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::INSCONFIG>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::INSCONFIG>& insconfig)
{
  assert(msg->getMessageId()== INSCONFIG_OEM7_MSGID);

  const INSCONFIG_FixedMem* insconfigmem =
      reinterpret_cast<const INSCONFIG_FixedMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  insconfig.reset(new novatel_oem7_msgs::msg::INSCONFIG);

  insconfig->imu_type                         = insconfigmem->imu_type;
  insconfig->mapping                          = insconfigmem->mapping;
  insconfig->initial_alignment_velocity       = insconfigmem->initial_alignment_velocity;
  insconfig->heave_window                     = insconfigmem->heave_window;
  insconfig->profile                          = insconfigmem->profile;

  std::copy(
      insconfigmem->enabled_updates,
      insconfigmem->enabled_updates + arr_size(insconfigmem->enabled_updates),
      insconfig->enabled_updates.begin());

  insconfig->alignment_mode.mode              = insconfigmem->alignment_mode;
  insconfig->relative_ins_output_frame.frame  = insconfigmem->relative_ins_output_frame;
  insconfig->relative_ins_output_direction   = insconfigmem->relative_ins_output_direction;

  std::copy(
      insconfigmem->ins_receiver_status,
      insconfigmem->ins_receiver_status + arr_size(insconfigmem->ins_receiver_status),
      insconfig->ins_receiver_status.status.begin());

  insconfig->ins_seed_enabled                 = insconfigmem->ins_seed_enabled;
  insconfig->ins_seed_validation              = insconfigmem->ins_seed_validation;
  insconfig->reserved_1 = insconfigmem->reserved_1;
  insconfig->reserved_2 = insconfigmem->reserved_2;
  insconfig->reserved_3 = insconfigmem->reserved_3;
  insconfig->reserved_4 = insconfigmem->reserved_4;
  insconfig->reserved_5 = insconfigmem->reserved_5;
  insconfig->reserved_6 = insconfigmem->reserved_6;
  insconfig->reserved_7 = insconfigmem->reserved_7;

  insconfig->translations.reserve(Get_INSCONFIG_NumTranslations(insconfigmem));
  for(size_t idx = 0;
             idx < Get_INSCONFIG_NumTranslations(insconfigmem);
             idx++)
  {
    const INSCONFIG_TranslationMem* trmem = Get_INSCONFIG_Translation(insconfigmem, idx);
    novatel_oem7_msgs::msg::Translation& tr = insconfig->translations[idx];

    tr.translation.type    = trmem->translation;
    tr.frame.frame         = trmem->frame;
    tr.x_offset            = trmem->x_offset;
    tr.y_offset            = trmem->y_offset;
    tr.z_offset            = trmem->z_offset;
    tr.x_uncertainty       = trmem->x_uncertainty;
    tr.y_uncertainty       = trmem->y_uncertainty;
    tr.z_uncertainty       = trmem->z_uncertainty;
    tr.translation_source.status  = trmem->translation_source;
  }

  insconfig->rotations.reserve(Get_INSCONFIG_NumRotations(insconfigmem));
  for(size_t idx = 0;
             idx < Get_INSCONFIG_NumRotations(insconfigmem);
             idx++)
  {
    const INSCONFIG_RotationMem* rtmem = Get_INSCONFIG_Rotation(insconfigmem, idx);
    novatel_oem7_msgs::msg::Rotation& rt = insconfig->rotations[idx];
    rt.rotation.offset         = rtmem->rotation;
    rt.frame.frame             = rtmem->frame;
    rt.x_rotation              = rtmem->x_rotation;
    rt.y_rotation              = rtmem->y_rotation;
    rt.z_rotation              = rtmem->z_rotation;
    rt.x_rotation_stdev        = rtmem->x_rotation_stdev;
    rt.y_rotation_stdev        = rtmem->y_rotation_stdev;
    rt.z_rotation_stdev        = rtmem->z_rotation_stdev;
    rt.rotation_source.status  = rtmem->rotation_source;
  }

  static const std::string name = "INSCONFIG";
  SetOem7Header(msg, name, insconfig->nov_header);
}


template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::INSPVAX>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::INSPVAX>& inspvax)
{
  assert(msg->getMessageId() == INSPVAX_OEM7_MSGID);

  const INSPVAXMem* mem = reinterpret_cast<const INSPVAXMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  inspvax.reset(new novatel_oem7_msgs::msg::INSPVAX);

  inspvax->ins_status.status        = mem->ins_status;
  inspvax->pos_type.type            = mem->pos_type;
  inspvax->latitude                 = mem->latitude;
  inspvax->longitude                = mem->longitude;
  inspvax->height                   = mem->height;
  inspvax->undulation               = mem->undulation;
  inspvax->north_velocity           = mem->north_velocity;
  inspvax->east_velocity            = mem->east_velocity;
  inspvax->up_velocity              = mem->up_velocity;
  inspvax->roll                     = mem->roll;
  inspvax->pitch                    = mem->pitch;
  inspvax->azimuth                  = mem->azimuth;
  inspvax->latitude_stdev           = mem->latitude_stdev;
  inspvax->longitude_stdev          = mem->longitude_stdev;
  inspvax->height_stdev             = mem->height_stdev;
  inspvax->north_velocity_stdev     = mem->north_velocity_stdev;
  inspvax->east_velocity_stdev      = mem->east_velocity_stdev;
  inspvax->up_velocity_stdev        = mem->up_velocity_stdev;
  inspvax->roll_stdev               = mem->roll_stdev;
  inspvax->pitch_stdev              = mem->pitch_stdev;
  inspvax->azimuth_stdev            = mem->azimuth_stdev;
  inspvax->ext_sol_status.status    = mem->extended_status;

  static const std::string name = "INSPVAX";
  SetOem7Header(msg, name, inspvax->nov_header);
}



template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::INSSTDEV>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::INSSTDEV>& insstdev)
{
  assert(msg->getMessageId() == INSSTDEV_OEM7_MSGID);

  const INSSTDEVMem* raw = reinterpret_cast<const INSSTDEVMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  insstdev.reset(new novatel_oem7_msgs::msg::INSSTDEV);

  insstdev->latitude_stdev         = raw->latitude_stdev;
  insstdev->longitude_stdev        = raw->longitude_stdev;
  insstdev->height_stdev           = raw->height_stdev;
  insstdev->north_velocity_stdev   = raw->north_velocity_stdev;
  insstdev->east_velocity_stdev    = raw->east_velocity_stdev;
  insstdev->up_velocity_stdev      = raw->up_velocity_stdev;
  insstdev->roll_stdev             = raw->roll_stdev;
  insstdev->pitch_stdev            = raw->pitch_stdev;
  insstdev->azimuth_stdev          = raw->azimuth_stdev;
  insstdev->ext_sol_status.status  = raw->ext_sol_status;
  insstdev->time_since_last_update = raw->time_since_last_update;
  insstdev->reserved1              = raw->reserved1;
  insstdev->reserved2              = raw->reserved2;
  insstdev->reserved3              = raw->reserved3;

  static const std::string name = "INSSTDEV";
  SetOem7Header(msg, name, insstdev->nov_header);
}

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::CORRIMU>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::CORRIMU>& corrimu)
{
  corrimu.reset(new novatel_oem7_msgs::msg::CORRIMU);

  if(msg->getMessageId() == CORRIMUS_OEM7_MSGID)
  {
    const CORRIMUSMem* raw = reinterpret_cast<const CORRIMUSMem*>(msg->getMessageData(OEM7_BINARY_MSG_SHORT_HDR_LEN));
    corrimu->imu_data_count   = raw->imu_data_count;
    corrimu->pitch_rate       = raw->pitch_rate;
    corrimu->roll_rate        = raw->roll_rate;
    corrimu->yaw_rate         = raw->yaw_rate;
    corrimu->lateral_acc      = raw->lateral_acc;
    corrimu->longitudinal_acc = raw->longitudinal_acc;
    corrimu->vertical_acc     = raw->vertical_acc;
  }
  else if(msg->getMessageId() == IMURATECORRIMUS_OEM7_MSGID)
  {
    const IMURATECORRIMUSMem* raw =
        reinterpret_cast<const IMURATECORRIMUSMem*>(msg->getMessageData(OEM7_BINARY_MSG_SHORT_HDR_LEN));
    corrimu->imu_data_count   = 1; 
    corrimu->pitch_rate       = raw->pitch_rate;
    corrimu->roll_rate        = raw->roll_rate;
    corrimu->yaw_rate         = raw->yaw_rate;
    corrimu->lateral_acc      = raw->lateral_acc;
    corrimu->longitudinal_acc = raw->longitudinal_acc;
    corrimu->vertical_acc     = raw->vertical_acc;
  }
  else
  {
    assert(false);
  }

  static const std::string name = "CORRIMU";
  SetOem7ShortHeader(msg, name, corrimu->nov_header);
}

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::TIME>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::TIME>& time)
{
  assert(msg->getMessageId()== TIME_OEM7_MSGID);

  const TIMEMem* mem = reinterpret_cast<const TIMEMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  time.reset(new novatel_oem7_msgs::msg::TIME);

  time->clock_status  = mem->clock_status;
  time->offset        = mem->offset;
  time->offset_std    = mem->offset_std;
  time->utc_offset    = mem->utc_offset;
  time->utc_year      = mem->utc_year;
  time->utc_month     = mem->utc_month;
  time->utc_day       = mem->utc_day;
  time->utc_hour      = mem->utc_hour;
  time->utc_min       = mem->utc_min;
  time->utc_msec      = mem->utc_msec;
  time->utc_status    = mem->utc_status;

  static const std::string name = "TIME";
  SetOem7Header(msg, name, time->nov_header);
}

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::RXSTATUS>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::RXSTATUS>& rxstatus)
{
  assert(msg->getMessageId() == RXSTATUS_OEM7_MSGID);

  const RXSTATUSMem* mem = reinterpret_cast<const RXSTATUSMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  rxstatus.reset(new novatel_oem7_msgs::msg::RXSTATUS);

  rxstatus->error              = mem->error;
  rxstatus->num_status_codes   = mem->num_status_codes;
  rxstatus->rxstat             = mem->rxstat;
  rxstatus->rxstat_pri_mask    = mem->rxstat_pri_mask;
  rxstatus->rxstat_set_mask    = mem->rxstat_set_mask;
  rxstatus->rxstat_clr_mask    = mem->rxstat_clr_mask;
  rxstatus->aux1_stat          = mem->aux1_stat;
  rxstatus->aux1_stat_pri      = mem->aux1_stat_pri;
  rxstatus->aux1_stat_set      = mem->aux1_stat_set;
  rxstatus->aux1_stat_clr      = mem->aux1_stat_clr;
  rxstatus->aux2_stat          = mem->aux2_stat;
  rxstatus->aux2_stat_pri      = mem->aux2_stat_pri;
  rxstatus->aux2_stat_set      = mem->aux2_stat_set;
  rxstatus->aux2_stat_clr      = mem->aux2_stat_clr;
  rxstatus->aux3_stat          = mem->aux3_stat;
  rxstatus->aux3_stat_pri      = mem->aux3_stat_pri;
  rxstatus->aux3_stat_set      = mem->aux3_stat_set;
  rxstatus->aux3_stat_clr      = mem->aux3_stat_clr;
  rxstatus->aux4_stat          = mem->aux4_stat;
  rxstatus->aux4_stat_pri      = mem->aux4_stat_pri;
  rxstatus->aux4_stat_set      = mem->aux4_stat_set;
  rxstatus->aux4_stat_clr      = mem->aux4_stat_clr;


  static const std::string name = "RXSTATUS";
  SetOem7Header(msg, name, rxstatus->nov_header);
};

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::PPPPOS>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::PPPPOS>& ppppos)
{
  assert(msg->getMessageId() == PPPPOS_OEM7_MSGID);

  const PPPPOSMem* pp = reinterpret_cast<const PPPPOSMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  ppppos.reset(new novatel_oem7_msgs::msg::PPPPOS);

  ppppos->sol_status.status      = pp->sol_stat;
  ppppos->pos_type.type          = pp->pos_type;
  ppppos->lat                    = pp->lat;
  ppppos->lon                    = pp->lon;
  ppppos->hgt                    = pp->hgt;
  ppppos->undulation             = pp->undulation;
  ppppos->datum_id               = pp->datum_id;
  ppppos->lat_stdev              = pp->lat_stdev;
  ppppos->lon_stdev              = pp->lon_stdev;
  ppppos->hgt_stdev              = pp->hgt_stdev;
  ppppos->stn_id.assign(           pp->stn_id, arr_size(pp->stn_id));
  ppppos->diff_age               = pp->diff_age;
  ppppos->sol_age                = pp->sol_age;
  ppppos->num_svs                = pp->num_svs;
  ppppos->num_sol_svs            = pp->num_sol_svs;
  ppppos->num_sol_l1_svs         = pp->num_sol_l1_svs;
  ppppos->num_sol_multi_svs      = pp->num_sol_multi_svs;
  ppppos->reserved               = pp->reserved;
  ppppos->ext_sol_stat.status    = pp->ext_sol_stat;
  ppppos->reserved2              = pp->reserved2;
  ppppos->gps_glonass_sig_mask   = pp->gps_glonass_sig_mask;

  static const std::string name = "PPPPOS";
  SetOem7Header(msg, name, ppppos->nov_header);
}

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::TERRASTARINFO>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::TERRASTARINFO>& terrastarinfo)
{
  assert(msg->getMessageId() == TERRASTARINFO_OEM7_MSGID);

  const TERRASTARINFOMem* tsi = reinterpret_cast<const TERRASTARINFOMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  terrastarinfo.reset(new novatel_oem7_msgs::msg::TERRASTARINFO);

  terrastarinfo->product_activation_code        = tsi->product_activation_code;
  terrastarinfo->sub_type.type                  = tsi->sub_type;
  terrastarinfo->sub_permission.permission      = tsi->sub_permission;
  terrastarinfo->service_end_day_of_year        = tsi->service_end_day_of_year;
  terrastarinfo->service_end_year               = tsi->service_end_year;
  terrastarinfo->reserved                       = tsi->reserved;
  terrastarinfo->region_restriction.restriction = tsi->region_restriction;
  terrastarinfo->center_point_latitude          = tsi->center_point_latitude;
  terrastarinfo->center_point_longitude         = tsi->center_point_longitude;
  terrastarinfo->radius                         = tsi->radius;

  static const std::string name = "TERRASTARINFO";
  SetOem7Header(msg, name, terrastarinfo->nov_header);
}

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::TERRASTARSTATUS>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::TERRASTARSTATUS>& terrastarstatus)
{
  assert(msg->getMessageId() == TERRASTARSTATUS_OEM7_MSGID);

  const TERRASTARSTATUSMem* tss = reinterpret_cast<const TERRASTARSTATUSMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  terrastarstatus.reset(new novatel_oem7_msgs::msg::TERRASTARSTATUS);

  terrastarstatus->access_status.status      = tss->access_status;
  terrastarstatus->sync_state.state          = tss->sync_state;
  terrastarstatus->reserved                  = tss->reserved;
  terrastarstatus->local_area_status.status  = tss->local_area_status;
  terrastarstatus->geo_status.status         = tss->geo_status;

  static const std::string name = "TERRASTARSTATUS";
  SetOem7Header(msg, name, terrastarstatus->nov_header);
}


template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&, std::shared_ptr<novatel_oem7_msgs::msg::BESTPOS>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&,  std::shared_ptr<novatel_oem7_msgs::msg::BESTVEL>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&,  std::shared_ptr<novatel_oem7_msgs::msg::BESTUTM>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&,  std::shared_ptr<novatel_oem7_msgs::msg::BESTGNSSPOS>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&,  std::shared_ptr<novatel_oem7_msgs::msg::INSPVA>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&, std::shared_ptr<novatel_oem7_msgs::msg::INSPVAX>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&,  std::shared_ptr<novatel_oem7_msgs::msg::INSCONFIG>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&,  std::shared_ptr<novatel_oem7_msgs::msg::INSSTDEV>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&,  std::shared_ptr<novatel_oem7_msgs::msg::CORRIMU>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&,  std::shared_ptr<novatel_oem7_msgs::msg::TIME>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&,  std::shared_ptr<novatel_oem7_msgs::msg::RXSTATUS>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&,  std::shared_ptr<novatel_oem7_msgs::msg::PPPPOS>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&,  std::shared_ptr<novatel_oem7_msgs::msg::TERRASTARSTATUS>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&,  std::shared_ptr<novatel_oem7_msgs::msg::TERRASTARINFO>&);


//---------------------------------------------------------------------------------------------------------------
/***
 * Obtains DOP values from Oem7 PSRDOP2 message
 */
void
GetDOPFromPSRDOP2(
    const Oem7RawMessageIf::ConstPtr& msg,
    uint32_t system_to_use,
    double&      gdop,
    double&      pdop,
    double&      hdop,
    double&      vdop,
    double&      tdop)
{
  const PSRDOP2_FixedMem* mem = reinterpret_cast<const PSRDOP2_FixedMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));

  gdop  = mem->gdop;
  pdop  = mem->pdop;
  hdop  = mem->hdop;
  vdop  = mem->vdop;

  const size_t num_sys = Get_PSRDOP2_NumSystems(mem);

  for(int i = 0;
          i < num_sys;
          i++)
  {
    const PSRDOP2_SystemMem* sys = Get_PSRDOP2_System(mem, i);
    if(sys->system == system_to_use)
    {
      tdop = sys->tdop;
      break;
    }
  }
}





}
