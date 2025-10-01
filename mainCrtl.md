@startuml
hide empty description

state CACHE_CRTL {
    [*] -[dashed]-> IDLE
    IDLE --> LOOKUP : in_segment & bus_idle
    note on link
        fetch entry from array
    end note
    IDLE --> PASS_THROUGH : !in_segment & bus_idle

    state PASS_THROUGH_RESPOND #line.dashed

    PASS_THROUGH --> PASS_THROUGH_RESPOND : bus_idle
    PASS_THROUGH_RESPOND -up[dashed]-> IDLE

    LOOKUP -left-> WRITE : mem_write_req
    WRITE -[dashed]-> PASS_THROUGH : !hit
    WRITE -[dashed]-> WRITE_UPDATE : hit

    state WRITE #line.dashed
    state WRITE_UPDATE #line.dashed

    WRITE_UPDATE -[dashed]-> WRITE_RESPOND
    note on link
        update local entry
    end note
    WRITE_RESPOND --> IDLE
    note on link
        write local to array
    end note

    LOOKUP -right-> READ : mem_read_req
    READ -[dashed]-> READ_RESPOND : hit
    READ -right[dashed]-> REFILL : !hit

    state READ #line.dashed

    READ_RESPOND --> IDLE

    REFILL_RESPOND -[dashed]-> REPLACE
    WRITE_BACK --> IDLE

    REFILL --> REFILL_RESPOND : first_word_read

    state REFILL_RESPOND #line.dashed
    state UPDATE_REFILL #line.dashed

    REPLACE --> UPDATE_REFILL : pol_ready & block_ready
    note on link
        update local entry
    end note
    UPDATE_REFILL -[dashed]-> IDLE : !need_wb
    note on link
        write local to array
    end note
    UPDATE_REFILL -[dashed]-> WRITE_BACK : need_wb
    note on link
        write local to array
    end note
}

state BUS_CRTL {
    state WAIT_CMD as "IDLE"
    state PUSH as "WRITE"
    state PULL as "READ"

    [*] --> WAIT_CMD
    WAIT_CMD --> PUSH : cmd_write
    WAIT_CMD --> PULL : cmd_read
    PUSH --> WAIT_CMD : finished
    PULL --> WAIT_CMD : finished
}

@enduml
