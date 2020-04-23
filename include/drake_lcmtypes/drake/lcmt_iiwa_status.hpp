/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#ifndef __drake_lcmt_iiwa_status_hpp__
#define __drake_lcmt_iiwa_status_hpp__

#include "lcm/lcm_coretypes.h"

#include <vector>

namespace drake
{

class lcmt_iiwa_status
{
    public:
        /// The timestamp in microseconds.
        int64_t    utime;

        int32_t    num_joints;

        /**
         * From FRI documentation: "The currently measured joint positions of the
         * robot in radians."
         */
        std::vector< double > joint_position_measured;

        /**
         * The FRI driver does not provide velocity; we estimate it in our driver via
         * a low-pass filter.  Units are radians / sec.
         */
        std::vector< double > joint_velocity_estimated;

        /**
         * From FRI documentation: "The last commanded joint positions of the robot in
         * radians."
         */
        std::vector< double > joint_position_commanded;

        /**
         * From FRI documentation:
         * "The joint positions commanded by the interpolator in radians. When
         * commanding a motion overlay in your robot application, this method will
         * give access to the joint positions currently commanded by the motion
         * interpolator.  This method will return NULL during monitoring mode."
         *
         * The Kuka motion interpolated code is a black-box to us, so we typically do
         * not try to model/simulate this signal.
         */
        std::vector< double > joint_position_ipo;

        /**
         * From FRI documentation: "The currently measured joint torques of the robot
         * in Nm."
         *
         * This appears to be the raw measurement of the torque sensors, which is
         * attempting to track joint_torque_commanded.
         */
        std::vector< double > joint_torque_measured;

        /**
         * From FRI documentation: "The last commanded joint torques of the robot in
         * Nm."
         *
         * This appears to be most similar to the torque input to multibody plant.
         */
        std::vector< double > joint_torque_commanded;

        /**
         * From FRI documentation: "The currently measured external joint torques of
         * the robot in Nm.  The external torques corresponds to the measured torques
         * when removing the torques induced by the robot itself."
         *
         * This appears to be the contact forces (in joint coordinates) as well as any
         * residuals from modeling errors (as computed by the onboard Kuka inverse
         * dynamics model, which is a black-box to us). Recall that the inertia of
         * the tool is included (potentially very approximately) in the onboard Kuka
         * model, so long as a tool is defined in the active Sunrise project. You can
         * use the Kuka pendant to teach the tool inertia.
         */
        std::vector< double > joint_torque_external;

    public:
        /**
         * Encode a message into binary form.
         *
         * @param buf The output buffer.
         * @param offset Encoding starts at thie byte offset into @p buf.
         * @param maxlen Maximum number of bytes to write.  This should generally be
         *  equal to getEncodedSize().
         * @return The number of bytes encoded, or <0 on error.
         */
        inline int encode(void *buf, int offset, int maxlen) const;

        /**
         * Check how many bytes are required to encode this message.
         */
        inline int getEncodedSize() const;

        /**
         * Decode a message from binary form into this instance.
         *
         * @param buf The buffer containing the encoded message.
         * @param offset The byte offset into @p buf where the encoded message starts.
         * @param maxlen The maximum number of bytes to read while decoding.
         * @return The number of bytes decoded, or <0 if an error occured.
         */
        inline int decode(const void *buf, int offset, int maxlen);

        /**
         * Retrieve the 64-bit fingerprint identifying the structure of the message.
         * Note that the fingerprint is the same for all instances of the same
         * message type, and is a fingerprint on the message type definition, not on
         * the message contents.
         */
        inline static int64_t getHash();

        /**
         * Returns "lcmt_iiwa_status"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int lcmt_iiwa_status::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int lcmt_iiwa_status::decode(const void *buf, int offset, int maxlen)
{
    int pos = 0, thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int lcmt_iiwa_status::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t lcmt_iiwa_status::getHash()
{
    static int64_t hash = static_cast<int64_t>(_computeHash(NULL));
    return hash;
}

const char* lcmt_iiwa_status::getTypeName()
{
    return "lcmt_iiwa_status";
}

int lcmt_iiwa_status::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->utime, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->num_joints, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->num_joints > 0) {
        tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->joint_position_measured[0], this->num_joints);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->num_joints > 0) {
        tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->joint_velocity_estimated[0], this->num_joints);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->num_joints > 0) {
        tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->joint_position_commanded[0], this->num_joints);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->num_joints > 0) {
        tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->joint_position_ipo[0], this->num_joints);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->num_joints > 0) {
        tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->joint_torque_measured[0], this->num_joints);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->num_joints > 0) {
        tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->joint_torque_commanded[0], this->num_joints);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->num_joints > 0) {
        tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->joint_torque_external[0], this->num_joints);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int lcmt_iiwa_status::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->utime, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->num_joints, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->num_joints) {
        this->joint_position_measured.resize(this->num_joints);
        tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->joint_position_measured[0], this->num_joints);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->num_joints) {
        this->joint_velocity_estimated.resize(this->num_joints);
        tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->joint_velocity_estimated[0], this->num_joints);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->num_joints) {
        this->joint_position_commanded.resize(this->num_joints);
        tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->joint_position_commanded[0], this->num_joints);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->num_joints) {
        this->joint_position_ipo.resize(this->num_joints);
        tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->joint_position_ipo[0], this->num_joints);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->num_joints) {
        this->joint_torque_measured.resize(this->num_joints);
        tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->joint_torque_measured[0], this->num_joints);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->num_joints) {
        this->joint_torque_commanded.resize(this->num_joints);
        tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->joint_torque_commanded[0], this->num_joints);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->num_joints) {
        this->joint_torque_external.resize(this->num_joints);
        tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->joint_torque_external[0], this->num_joints);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int lcmt_iiwa_status::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, this->num_joints);
    enc_size += __double_encoded_array_size(NULL, this->num_joints);
    enc_size += __double_encoded_array_size(NULL, this->num_joints);
    enc_size += __double_encoded_array_size(NULL, this->num_joints);
    enc_size += __double_encoded_array_size(NULL, this->num_joints);
    enc_size += __double_encoded_array_size(NULL, this->num_joints);
    enc_size += __double_encoded_array_size(NULL, this->num_joints);
    return enc_size;
}

uint64_t lcmt_iiwa_status::_computeHash(const __lcm_hash_ptr *)
{
    uint64_t hash = 0xf8b7dc0214255e51LL;
    return (hash<<1) + ((hash>>63)&1);
}

}

#endif
