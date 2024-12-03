@startuml
hide empty description

[*] --> IDLE
IDLE --> ADDR_DECODE : mem_read_req | mem_write_req

note on link
    pseudo-transition
end note

ADDR_DECODE --> PASS_THROUGH : !in_segment
PASS_THROUGH --> PASS_THROUGH_RESPOND
PASS_THROUGH_RESPOND --> IDLE

ADDR_DECODE --> WRITE : mem_write_req & in_segment
state WRITE {
    [*] --> PASS_THROUGH : !cache_hit
    note on link
        pseudo-transition
    end note
    [*] --> WRITE_UPDATE  : cache_hit
    note on link
        pseudo-transition
    end note
    WRITE_UPDATE --> WRITE_RESPOND
    note on link
        pseudo-transition
    end note
    WRITE_RESPOND --> [*]
}
WRITE --> IDLE
note on link
    pseudo-transition
end note

ADDR_DECODE --> READ : mem_read_req & in_segment
note on link
    priority if mem_read_req & mem_write_req
end note
state READ {
    [*] --> READ_RESPOND : cache_hit
    note on link
        pseudo-transition
    end note
    [*] --> REFILL : !cache_hit
    note on link
        pseudo-transition
    end note
    state REFILL {
        [*] --> FETCH
        FETCH --> [*]
        ||
        [*] --> GET_CANDIDATE
        GET_CANDIDATE --> [*]
    }
    REFILL --> REPLACE
    REPLACE --> READ_RESPOND
    READ_RESPOND --> [*] : !need_write_back
    READ_RESPOND --> WRITE_BACK : need_write_back
    WRITE_BACK --> [*]
}
READ --> IDLE
note on link
    pseudo-transition
end note

@enduml
