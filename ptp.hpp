#ifndef PTP_H__
#define PTP_H__

#include <cstdint>
#include <array>
#include <boost/multiprecision/cpp_int.hpp>

namespace mp = boost::multiprecision;

namespace PTP {

struct UInteger8 { 
    std::uint8_t raw_value[1];
    std::uint8_t get() const { return this->raw_value[0]; }
    void set(std::uint8_t value) { this->raw_value[0] = value; }
    operator std::uint8_t() const { return this->get(); }
    UInteger8& operator=(std::uint8_t value) { this->set(value); return *this; }
};
struct UInteger16 { 
    std::uint8_t raw_value[2];
    std::uint16_t get() const { return (this->raw_value[0] << 8) | this->raw_value[1]; }
    void set(std::uint16_t value) { this->raw_value[0] = value >> 8; this->raw_value[1] = value; }
    operator std::uint16_t() const { return this->get(); }
    UInteger16& operator=(std::uint16_t value) { this->set(value); return *this; }
};
struct UInteger32 { 
    std::uint8_t raw_value[4];
    UInteger32() = default;
    UInteger32(const UInteger32&) = default;
    UInteger32(std::uint32_t value) {
        this->set(value);
    }
    std::uint32_t get() const { return (this->raw_value[0] << 24) | (this->raw_value[1]  << 16) | (this->raw_value[2] << 8) | this->raw_value[3]; }
    void set(std::uint32_t value) { this->raw_value[0] = value >> 24; this->raw_value[1] = value >> 16; this->raw_value[2] = value >> 8; this->raw_value[3] = value; }
    operator std::uint32_t() const { return this->get(); }
    UInteger32& operator=(std::uint32_t value) { this->set(value); return *this; }
};
struct UInteger48 { 
    std::uint8_t raw_value[6];
    UInteger48() = default;
    UInteger48(const UInteger48&) = default;
    UInteger48(std::uint64_t value) {
        this->set(value);
    }
    std::uint64_t get() const { 
        return (static_cast<std::uint64_t>(this->raw_value[0]) << 40) | 
               (static_cast<std::uint64_t>(this->raw_value[1]) << 32) | 
               (static_cast<std::uint64_t>(this->raw_value[2]) << 24) | 
               (static_cast<std::uint64_t>(this->raw_value[3]) << 16) | 
               (static_cast<std::uint64_t>(this->raw_value[4]) <<  8) | 
               this->raw_value[5]; 
        }
    void set(std::uint64_t value) { 
        this->raw_value[0] = value >> 40; 
        this->raw_value[1] = value >> 32; 
        this->raw_value[2] = value >> 24; 
        this->raw_value[3] = value >> 16; 
        this->raw_value[4] = value >> 8; 
        this->raw_value[5] = value; 
    }
    operator std::uint64_t() const { return this->get(); }
    UInteger48& operator=(std::uint64_t value) { this->set(value); return *this; }
};
struct UInteger64 { 
    std::uint8_t raw_value[8];
    std::uint64_t get() const { 
        return (static_cast<std::uint64_t>(this->raw_value[0]) << 56) | 
               (static_cast<std::uint64_t>(this->raw_value[1]) << 48) | 
               (static_cast<std::uint64_t>(this->raw_value[2]) << 40) | 
               (static_cast<std::uint64_t>(this->raw_value[3]) << 32) | 
               (static_cast<std::uint64_t>(this->raw_value[4]) << 24) | 
               (static_cast<std::uint64_t>(this->raw_value[5]) << 16) | 
               (static_cast<std::uint64_t>(this->raw_value[6]) <<  8) | 
               this->raw_value[7]; 
        }
    void set(std::uint64_t value) { 
        this->raw_value[0] = value >> 56; 
        this->raw_value[1] = value >> 48; 
        this->raw_value[2] = value >> 40; 
        this->raw_value[3] = value >> 32; 
        this->raw_value[4] = value >> 24; 
        this->raw_value[5] = value >> 16; 
        this->raw_value[6] = value >> 8; 
        this->raw_value[7] = value; 
    }
    operator std::uint64_t() const { return this->get(); }
    UInteger64& operator=(std::uint64_t value) { this->set(value); return *this; }
};
struct Integer16 {
    UInteger16 raw_value;
    std::int16_t get() const { return this->raw_value.get(); }
    void set(std::int16_t value) { this->raw_value.set(static_cast<std::uint16_t>(value)); }
    operator std::int16_t() const { return this->get(); }
    Integer16& operator=(std::int16_t value) { this->set(value); return *this; }
};
struct Integer48 { 
    std::uint8_t raw_value[6];
    std::int64_t get() const { 
        std::uint64_t raw = (static_cast<std::uint64_t>(this->raw_value[0]) << 40) | 
                            (static_cast<std::uint64_t>(this->raw_value[1]) << 32) | 
                            (static_cast<std::uint64_t>(this->raw_value[2]) << 24) | 
                            (static_cast<std::uint64_t>(this->raw_value[3]) << 16) | 
                            (static_cast<std::uint64_t>(this->raw_value[4]) <<  8) | 
                            this->raw_value[5]; 
        return static_cast<std::int64_t>( (raw & 0x800000000000ul) ? (~raw + 1) : raw);
    }
    void set(std::int64_t value) { 
        std::uint64_t v = static_cast<std::uint64_t>(value);
        this->raw_value[0] = v >> 40; 
        this->raw_value[1] = v >> 32; 
        this->raw_value[2] = v >> 24; 
        this->raw_value[3] = v >> 16; 
        this->raw_value[4] = v >> 8; 
        this->raw_value[5] = v; 
    }
    operator std::int64_t() const { return this->get(); }
    Integer48& operator=(std::int64_t value) { this->set(value); return *this; }
};

struct Timestamp {
    UInteger48 seconds_field;
    UInteger32 nanoseconds_field;
};

struct TimeDiff {
    mp::int128_t raw;

    TimeDiff() = default;
    TimeDiff(const TimeDiff&) = default;
    TimeDiff(const mp::int128_t& value) : raw(value) {}

    TimeDiff operator-() const { return TimeDiff(-this->raw); }
    TimeDiff& operator=(const TimeDiff& rhs) { this->raw = rhs.raw; return *this; }
    TimeDiff operator+(const TimeDiff& rhs) const { return TimeDiff(this->raw + rhs.raw); }
    TimeDiff operator-(const TimeDiff& rhs) const { return TimeDiff(this->raw - rhs.raw); }
    TimeDiff operator*(const mp::int128_t& rhs) const { return TimeDiff(this->raw*rhs); }
    TimeDiff operator*(int64_t rhs) const { return TimeDiff(this->raw*rhs); }
    TimeDiff operator/(const mp::int128_t& rhs) const { return TimeDiff(this->raw/rhs); }
    TimeDiff operator/(int64_t rhs) const { return TimeDiff(this->raw/rhs); }
    TimeDiff operator%(const mp::int128_t& rhs) const { return TimeDiff(this->raw%rhs); }
    TimeDiff operator%(int64_t rhs) const { return TimeDiff(this->raw%rhs); }
    bool operator<(const TimeDiff& rhs) const { return this->raw < rhs.raw; }
    bool operator==(const TimeDiff& rhs) const { return this->raw == rhs.raw; }
    bool operator>(const TimeDiff& rhs) const { return this->raw > rhs.raw; }
};

struct Timestamp128 {
    mp::int128_t raw;
    
    Timestamp128() = default;
    Timestamp128(const mp::int128_t& value) : raw(value) {}
    Timestamp128(const Timestamp timestamp) {
        mp::int128_t seconds = timestamp.seconds_field.get();
        mp::int128_t nanoseconds = timestamp.nanoseconds_field.get();
        this->raw = seconds*1000000000ul + nanoseconds;
    }

    operator Timestamp() {
        return Timestamp {
            UInteger48(static_cast<uint64_t>(this->raw / 1000000000ul)),
            UInteger32(static_cast<uint64_t>(this->raw % 1000000000ul))
        };
    }

    Timestamp128& operator=(const Timestamp128& rhs) {
        this->raw = rhs.raw;
        return *this;
    }

    Timestamp128 operator+(const TimeDiff& rhs) {
        return Timestamp128(this->raw + rhs.raw);
    }
    TimeDiff operator-(const Timestamp128& rhs) {
        return TimeDiff(this->raw - rhs.raw);
    }
};

struct ClockIdentity {
    std::uint8_t values[8];
};
struct PortIdentity {
    ClockIdentity clock_identity;
    UInteger16 port_number;
};
struct PortAddressHeader {
    UInteger16 network_prootocol;
    UInteger16 address_length;
    std::uint8_t address_field[1];
};

struct FlagField {
    static constexpr std::uint16_t AlternateMasterFlag   = (1u << 0);
    static constexpr std::uint16_t TwoStepFlag           = (1u << 1);
    static constexpr std::uint16_t UnicastFlag           = (1u << 2);
    static constexpr std::uint16_t PTPProfileSpecific1   = (1u << 5);
    static constexpr std::uint16_t PTPProfileSpecific2   = (1u << 6);
    static constexpr std::uint16_t Leap61                = (1u << 8);
    static constexpr std::uint16_t Leap59                = (1u << 9);
    static constexpr std::uint16_t CurrentUtcOffsetValid = (1u << 10);
    static constexpr std::uint16_t PTPTimescale          = (1u << 11);
    static constexpr std::uint16_t TimeTraceable         = (1u << 12);
    static constexpr std::uint16_t FrequencyTraceable    = (1u << 13);

    std::uint16_t flags;

    bool get(std::uint16_t flag) const { return this->flags & flag; }
    void set(std::uint16_t flag) { this->flags |= flag; }
    void clear(std::uint16_t flag) { this->flags &= ~flag; }
};

enum class MessageType : std::uint8_t {
    Sync = 0,
    Delay_Req = 1,
    Pdelay_Req = 2,
    Pdelay_Resp  = 3,
    Follow_Up = 8,
    Delay_Resp = 9,
    Pdelay_Resp_Follow_Up = 10,
    Announce = 11,
    Signaling = 12,
    Management = 13,
};

struct NibbleRef {
    std::uint8_t& ref;
    bool upper;
    NibbleRef(std::uint8_t& ref, bool upper) : ref(ref), upper(upper) {}
    operator std::uint8_t() const { return this->upper ? this->ref >> 4 : this->ref & 0x0fu; }
    NibbleRef& operator=(std::uint8_t value) {
        if( this->upper ) {
            this->ref = (this->ref & 0x0fu) | (value << 4);
        }
        else {
            this->ref = (this->ref & 0xf0u) | (value & 0x0fu);
        }
        return *this;
    }
};
struct Nibbles {
    std::uint8_t value;
    Nibbles() = default;
    Nibbles(std::uint8_t value) : value(value) {}
    Nibbles(std::uint8_t upper, std::uint8_t lower) : value((upper << 4) | (lower & 0x0fu)) {}
    NibbleRef upper() { return NibbleRef(this->value, true); }
    NibbleRef lower() { return NibbleRef(this->value, false); }
    std::uint8_t const_upper() const { return this->value >> 4; }
    std::uint8_t const_lower() const { return this->value & 0x0fu; }
};

struct TransportSpecificMessageType {
    Nibbles nibbles;
    NibbleRef transport_specific() { return this->nibbles.upper(); }
    NibbleRef message_type()       { return this->nibbles.lower(); }
    std::uint8_t transport_specific_const() const { return this->nibbles.const_upper(); }
    MessageType message_type_const() const { return static_cast<MessageType>(this->nibbles.const_lower()); }
};
struct ReservedVersionPTP {
    Nibbles nibbles;
    NibbleRef reserved() { return this->nibbles.upper(); }
    NibbleRef version_ptp()       { return this->nibbles.lower(); }
    std::uint8_t reserved_const() const { return this->nibbles.const_upper(); }
    std::uint8_t version_ptp_const() const { return this->nibbles.const_lower(); }
};



struct MessageHeader {
    TransportSpecificMessageType transport_specific_message_type;
    ReservedVersionPTP reserved_version_ptp;
    UInteger16 message_length;
    UInteger8 domain_number;
    UInteger8 reserved0;
    FlagField flag_field;
    UInteger64 correction_field;
    UInteger32 reserved1;
    PortIdentity source_port_identity;
    UInteger16 sequence_id;
    UInteger8 control_field;
    UInteger8 log_message_interval;
};

struct SyncMessage {
    MessageHeader header;
    Timestamp origin_timestamp;
};

struct DelayReqMessage {
    MessageHeader header;
    Timestamp origin_timestamp;
};

struct FollowUpMessage {
    MessageHeader header;
    Timestamp precise_origin_timestamp;
};

struct DelayRespMessage {
    MessageHeader header;
    Timestamp receive_timestamp;
    PortIdentity requesting_port_identity;
};


struct NullLogger {
    template <typename... Args> static void info(const char* fmt, Args... args) {}
    template <typename... Args> static void error(const char* fmt, Args... args) {}
};

template<typename Transport, typename ClockSource, std::size_t MaxSlaves, typename Logger=NullLogger>
class MasterClock {
public:
    typedef typename Transport::AddressType AddressType;
private:
    Transport& transport;
    ClockSource& clock_source;

    enum class SlaveState {
        NotSyncing,
        Syncing,
        Synced,
    };
    struct SlaveContext {
        AddressType address;
        Timestamp128 last_sync_time;
        SlaveState state;
        std::uint16_t sequence_id;
    };
    std::size_t number_of_slaves;
    std::array<SlaveContext, MaxSlaves> slaves;

    SlaveContext* get_slave(const AddressType& address) {
        for(std::size_t i = 0; i < this->number_of_slaves; ++i) {
            if( this->slaves[i].address == address ) {
                return &this->slaves[i];
            }
        }
        return nullptr;
    }
public:
    MasterClock(Transport& transport, ClockSource& clock_source) : transport(transport), clock_source(clock_source), number_of_slaves(0) {}
    void add_slave(const AddressType& address) {
        if( number_of_slaves < MaxSlaves ) {
            auto& slave = this->slaves[this->number_of_slaves++];
            slave.address = address;
            slave.last_sync_time = static_cast<Timestamp128>(0);
            slave.state = SlaveState::NotSyncing;
            slave.sequence_id = 0;
        }
    }
    void process_response() {
        while(true) {
            DelayReqMessage response;
            AddressType address;
            int rc = this->transport.receive_packet(address, &response, sizeof(response));
            if( rc == sizeof(response) && response.header.transport_specific_message_type.message_type_const() == MessageType::Delay_Req) {
                Logger::info("[MASTER] Receive Delay_Req\n");
                auto slave = this->get_slave(address);
                if( slave != nullptr && slave->state == SlaveState::Syncing ) {
                    DelayRespMessage message;
                    message.header.transport_specific_message_type.message_type() = static_cast<std::uint8_t>(MessageType::Delay_Resp);
                    message.header.transport_specific_message_type.transport_specific() = 0;
                    message.header.reserved_version_ptp.version_ptp() = 1;
                    message.header.reserved_version_ptp.reserved() = 0;
                    message.header.message_length = sizeof(message) - sizeof(MessageHeader);
                    message.header.domain_number = 0;
                    message.header.flag_field.flags = FlagField::UnicastFlag;
                    message.header.sequence_id = slave->sequence_id++;
                    message.receive_timestamp = this->clock_source.now();
                    this->transport.send_packet(address, &message, sizeof(message));
                    Logger::info("[MASTER] Send Delay_Resp\n");
                    slave->state = SlaveState::Synced;
                    slave->last_sync_time = this->clock_source.now();
                }
            }
            else {
                break;
            }
        }
    }
    void update() {
        this->process_response();
        for(std::size_t i = 0; i < this->number_of_slaves; ++i) {
            auto& slave = this->slaves[i];
            switch(slave.state) {
            case SlaveState::Synced:
                auto duration = this->clock_source.now() - slave.last_sync_time;
                if( duration > TimeDiff(1000000000ul) ) {
                    slave.state = SlaveState::NotSyncing;
                }
                break;
            }
        }
        for(std::size_t i = 0; i < this->number_of_slaves; ++i) {
            auto& slave = this->slaves[i];
            switch(slave.state) {
            case SlaveState::NotSyncing: {
                SyncMessage message;
                message.header.transport_specific_message_type.message_type() = static_cast<std::uint8_t>(MessageType::Sync);
                message.header.transport_specific_message_type.transport_specific() = 0;
                message.header.reserved_version_ptp.version_ptp() = 1;
                message.header.reserved_version_ptp.reserved() = 0;
                message.header.message_length = sizeof(message) - sizeof(MessageHeader);
                message.header.domain_number = 0;
                message.header.flag_field.flags = FlagField::UnicastFlag;
                message.header.sequence_id = slave.sequence_id++;
                auto timestamp = this->clock_source.now();
                message.origin_timestamp = timestamp;
                if( this->transport.send_packet(slave.address, &message, sizeof(message)) ) {
                    Logger::info("[MASTER] Send sync\n");
                    slave.state = SlaveState::Syncing;
                    slave.last_sync_time = timestamp;
                    this->process_response();
                }
                break;
            }
            case SlaveState::Syncing: {
                if( this->clock_source.now() - slave.last_sync_time > TimeDiff(1000000000ul) ) {
                    // Timed out
                    slave.state = SlaveState::NotSyncing;
                }
                break;
            }
            }
        }
    }
};


template<typename Transport, typename UpdatableClockSource, typename Logger=NullLogger>
class SlaveClock {
public:
    typedef typename Transport::AddressType AddressType;
private:
    Transport& transport;
    UpdatableClockSource& clock_source;
    enum class SlaveState {
        Idle,
        SyncReceived,
        DelayReqSent,
    };

    SlaveState state;
    Timestamp128 timeSyncSent;
    Timestamp128 timeSyncReceived;
    Timestamp128 timeDelayReqSent;
    Timestamp128 timeDelayReqReceived;
    AddressType master_address;


    bool receive_sync() {
        AddressType address;
        SyncMessage message;
        int rc = this->transport.receive_packet(address, &message, sizeof(message));
        if( rc == sizeof(message) && message.header.transport_specific_message_type.message_type_const() == MessageType::Sync ) {
            this->timeSyncReceived = this->clock_source.now();
            this->timeSyncSent = message.origin_timestamp;
            this->master_address = address;
            this->state = SlaveState::SyncReceived;
            Logger::info("[SLAVE] Receive sync\n");
            return true;
        }
        else if( rc == 0 ) {
            return false;
        }
        else {
            Logger::error("[SLAVE] failed to receive packet: %d\n", errno);
            return false;
        }
    }
    bool send_delay_req() {
        DelayReqMessage message;
        message.header.transport_specific_message_type.message_type() = static_cast<std::uint8_t>(MessageType::Delay_Req);
        message.header.transport_specific_message_type.transport_specific() = 0;
        message.header.reserved_version_ptp.version_ptp() = 1;
        message.header.reserved_version_ptp.reserved() = 0;
        message.header.message_length = sizeof(message) - sizeof(MessageHeader);
        message.header.domain_number = 0;
        message.header.flag_field.flags = FlagField::UnicastFlag;
        message.header.sequence_id = 0;
        Timestamp128 timeDelayReqSent = this->clock_source.now();
        message.origin_timestamp = timeDelayReqSent;
        if( this->transport.send_packet(this->master_address, &message, sizeof(message)) ) {
            this->state = SlaveState::DelayReqSent;
            this->timeDelayReqSent = timeDelayReqSent;
            Logger::info("[SLAVE] Send Delay_Req\n");
            return true;
        }
        return false;
    }
    bool receive_delay_resp() {
        AddressType address;
        DelayRespMessage message;
        int rc = this->transport.receive_packet(address, &message, sizeof(message));
        if( rc == sizeof(message) && message.header.transport_specific_message_type.message_type_const() == MessageType::Delay_Resp ) {
            this->timeDelayReqReceived = message.receive_timestamp;
            auto offset = ((this->timeSyncReceived - this->timeSyncSent) - (this->timeDelayReqReceived - this->timeDelayReqSent)) / 2;
            Logger::info("[SLAVE] Receive Delay_Resp\n", offset.raw.str().c_str());
            Logger::info("[SLAVE] Sync     Sent: %s\n", this->timeSyncSent.raw.str().c_str());
            Logger::info("[SLAVE] Sync     Rcvd: %s\n", this->timeSyncReceived.raw.str().c_str());
            Logger::info("[SLAVE] DelayReq Sent: %s\n", this->timeDelayReqSent.raw.str().c_str());
            Logger::info("[SLAVE] DelayReq Rcvd: %s\n", this->timeDelayReqReceived.raw.str().c_str());
            Logger::info("[SLAVE] Offset: %s\n", offset.raw.str().c_str());

            this->clock_source.add_offset(-offset);
            this->state = SlaveState::Idle;
            
            return false;
        }
        else {
            return false;
        }
    }

public:
    SlaveClock(Transport& transport, UpdatableClockSource& clock_source) : transport(transport), clock_source(clock_source), state(SlaveState::Idle) {}
    
    bool update() {
        bool process_next = true;

        while(process_next) {
            switch(this->state) {
            case SlaveState::Idle:
                process_next = this->receive_sync();
                break;   
            case SlaveState::SyncReceived:
                process_next = this->send_delay_req();
                break;
            case SlaveState::DelayReqSent:
                process_next = this->receive_delay_resp();
                if( this->state == SlaveState::Idle && !process_next ) {
                    return true;
                }
                break;
            }
        }

        return false;
    }
};

};

#endif // PTP_H__