function(board_test TEST TARGET)
	#message("*** Target: ${TARGET}")
	# check if target exists
	if(TARGET ${TARGET})
		string(REGEX REPLACE ".*\\:" "" BOARD ${TARGET})
		set(NAME "${TEST}-${BOARD}")
		message("*** Board: ${BOARD}")

		add_executable(${NAME}
			${TEST}.cpp
		)
		target_include_directories(${NAME}
			PRIVATE
				../
				${BOARD}
		)
		target_link_libraries(${NAME}
			${TARGET}
			coco-spi::coco-spi
			coco-i2c::coco-i2c
			coco-mono-display::coco-mono-display
			${PROJECT_NAME}
		)

		# generate hex file for flashing the target
		if(${CMAKE_CROSSCOMPILING})
			#message("*** Generate Hex for ${NAME} using ${CMAKE_OBJCOPY}")
			add_custom_command(TARGET ${NAME}
				POST_BUILD
				COMMAND ${CMAKE_OBJCOPY} -O ihex ${NAME} ${NAME}.hex
			)
		endif()
	endif()
endfunction()

board_test(BME680Test coco-devboards::native)

board_test(BME680DisplayTest coco-devboards::emu)
board_test(BME680DisplayTest coco-devboards::stm32g431nucleo)
