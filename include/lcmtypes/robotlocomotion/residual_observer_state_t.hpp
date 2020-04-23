/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#ifndef __robotlocomotion_residual_observer_state_t_hpp__
#define __robotlocomotion_residual_observer_state_t_hpp__

#include "lcm/lcm_coretypes.h"

#include <vector>
#include <string>

namespace robotlocomotion
{

/**
 * This message type is intented to be used with the 
 * residual detector algorithm. The residual value is
 * recorded in the residual field. The other fields are 
 * for debugging purposes and can be ignored.
 */
class residual_observer_state_t
{
    public:
        int64_t    utime;

        int16_t    num_joints;

        std::vector< std::string > joint_name;

        std::vector< float > residual;

        std::vector< float > gravity;

        std::vector< float > internal_torque;

        std::vector< float > foot_contact_torque;

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
         * Returns "residual_observer_state_t"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int residual_observer_state_t::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int residual_observer_state_t::decode(const void *buf, int offset, int maxlen)
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

int residual_observer_state_t::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t residual_observer_state_t::getHash()
{
    static int64_t hash = static_cast<int64_t>(_computeHash(NULL));
    return hash;
}

const char* residual_observer_state_t::getTypeName()
{
    return "residual_observer_state_t";
}

int residual_observer_state_t::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->utime, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->num_joints, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    for (int a0 = 0; a0 < this->num_joints; a0++) {
        char* __cstr = const_cast<char*>(this->joint_name[a0].c_str());
        tlen = __string_encode_array(
            buf, offset + pos, maxlen - pos, &__cstr, 1);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->num_joints > 0) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->residual[0], this->num_joints);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->num_joints > 0) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->gravity[0], this->num_joints);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->num_joints > 0) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->internal_torque[0], this->num_joints);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->num_joints > 0) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->foot_contact_torque[0], this->num_joints);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int residual_observer_state_t::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->utime, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->num_joints, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    try {
        this->joint_name.resize(this->num_joints);
    } catch (...) {
        return -1;
    }
    for (int a0 = 0; a0 < this->num_joints; a0++) {
        int32_t __elem_len;
        tlen = __int32_t_decode_array(
            buf, offset + pos, maxlen - pos, &__elem_len, 1);
        if(tlen < 0) return tlen; else pos += tlen;
        if(__elem_len > maxlen - pos) return -1;
        this->joint_name[a0].assign(static_cast<const char*>(buf) + offset + pos, __elem_len -  1);
        pos += __elem_len;
    }

    if(this->num_joints) {
        this->residual.resize(this->num_joints);
        tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->residual[0], this->num_joints);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->num_joints) {
        this->gravity.resize(this->num_joints);
        tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->gravity[0], this->num_joints);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->num_joints) {
        this->internal_torque.resize(this->num_joints);
        tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->internal_torque[0], this->num_joints);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->num_joints) {
        this->foot_contact_torque.resize(this->num_joints);
        tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->foot_contact_torque[0], this->num_joints);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int residual_observer_state_t::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __int16_t_encoded_array_size(NULL, 1);
    for (int a0 = 0; a0 < this->num_joints; a0++) {
        enc_size += this->joint_name[a0].size() + 4 + 1;
    }
    enc_size += __float_encoded_array_size(NULL, this->num_joints);
    enc_size += __float_encoded_array_size(NULL, this->num_joints);
    enc_size += __float_encoded_array_size(NULL, this->num_joints);
    enc_size += __float_encoded_array_size(NULL, this->num_joints);
    return enc_size;
}

uint64_t residual_observer_state_t::_computeHash(const __lcm_hash_ptr *)
{
    uint64_t hash = 0x8c1b4e93b8978c7dLL;
    return (hash<<1) + ((hash>>63)&1);
}

}

#endif