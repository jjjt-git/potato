package tracing_types is
	type active_policy_t is (
		POL_RANDOM, POL_FIFO,
		POL_DLFU,
		POL_LRU, POL_MRU,
		POL_NONE
	);
	type policy_t is record
		random, fifo, lru, mru, dlfu: integer range 0 to 63;
		active: active_policy_t;
	end record;
end package tracing_types;