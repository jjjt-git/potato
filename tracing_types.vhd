package tracing_types is
	type policy_t is record
		random, fifo, lru, mru, dlfu: integer range 0 to 63;
	end record;
end package tracing_types;