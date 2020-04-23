/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#ifndef __robotlocomotion_plan_status_t_hpp__
#define __robotlocomotion_plan_status_t_hpp__

#include "lcm/lcm_coretypes.h"


namespace robotlocomotion
{

class plan_status_t
{
    public:
        int64_t    utime;

        int8_t     execution_status;

        /// Timestamp of the last plan message received
        int64_t    last_plan_msg_utime;

        /// Time at which the last plan actually began executing
        int64_t    last_plan_start_utime;

        int8_t     plan_type;

        int8_t     recovery_enabled;

        int8_t     bracing_enabled;

    public:
        static constexpr int8_t   EXECUTION_STATUS_EXECUTING = 0;
        static constexpr int8_t   EXECUTION_STATUS_FINISHED = 1;
        static constexpr int8_t   EXECUTION_STATUS_NO_PLAN = 2;
        static constexpr int8_t   UNKNOWN = 0;
        static constexpr int8_t   STANDING = 1;
        static constexpr int8_t   WALKING = 2;
        static constexpr int8_t   HARNESSED = 3;
        static constexpr int8_t   QUASISTATIC = 4;
        static constexpr int8_t   BRACING = 5;
        static constexpr int8_t   CRAWLING = 6;
        static constexpr int8_t   DUMMY = 7;
        static constexpr int8_t   MANIPULATING = 8;
        static constexpr int8_t   RECOVERING = 9;

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
         * Returns "plan_status_t"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int plan_status_t::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int plan_status_t::decode(const void *buf, int offset, int maxlen)
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

int plan_status_t::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t plan_status_t::getHash()
{
    static int64_t hash = static_cast<int64_t>(_computeHash(NULL));
    return hash;
}

const char* plan_status_t::getTypeName()
{
    return "plan_status_t";
}

int plan_status_t::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->utime, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &this->execution_status, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->last_plan_msg_utime, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->last_plan_start_utime, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &this->plan_type, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __boolean_encode_array(buf, offset + pos, maxlen - pos, &this->recovery_enabled, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __boolean_encode_array(buf, offset + pos, maxlen - pos, &this->bracing_enabled, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int plan_status_t::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->utime, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &this->execution_status, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->last_plan_msg_utime, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->last_plan_start_utime, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &this->plan_type, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __boolean_decode_array(buf, offset + pos, maxlen - pos, &this->recovery_enabled, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __boolean_decode_array(buf, offset + pos, maxlen - pos, &this->bracing_enabled, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int plan_status_t::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __int8_t_encoded_array_size(NULL, 1);
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __int8_t_encoded_array_size(NULL, 1);
    enc_size += __boolean_encoded_array_size(NULL, 1);
    enc_size += __boolean_encoded_array_size(NULL, 1);
    return enc_size;
}

uint64_t plan_status_t::_computeHash(const __lcm_hash_ptr *)
{
    uint64_t hash = 0xf946fe88ee1f80d4LL;
    return (hash<<1) + ((hash>>63)&1);
}

}

#endif