set(PanTilt_TESTS_FILES
    main.cpp
    PanTiltTests.cpp
)

add_executable(ut-panTilt ${PanTilt_TESTS_FILES})
target_link_libraries(
	ut-panTilt
	gmock
	CRFMocks
	PanTilt
	Logger)

add_test(PanTiltTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-panTilt)
