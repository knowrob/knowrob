include(CMakeParseArguments)

# TODO: there is a similar macros called FetchContent_* shipped with CMake.
#   Have a look if install_ontology can use it or be replaced with it.

macro(install_ontology)
	# parse options
	set(prefix ARG_)
	set(options OPTION)
	set(oneValueArgs URL FILE_NAME VERSION DESTINATION)
	set(multiValueArgs MULTIVALUE)
	cmake_parse_arguments("${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
	
	# input validation
	if(NOT ${prefix}_URL)
		message(FATAL_ERROR "URL option is required but missing.")
	endif()
	if(NOT ${prefix}_VERSION)
		message(FATAL_ERROR "VERSION option is required but missing.")
	endif()
	
	# get fallback values
	if(NOT ${prefix}_FILE_NAME)
		get_filename_component(${prefix}_FILE_NAME ${${prefix}_URL} NAME)
	endif()
	if(NOT ${prefix}_DESTINATION)
		set(${prefix}_DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/owl/external")
	endif()
	
	# strip the file extension
	get_filename_component(ontologyName ${${prefix}_FILE_NAME} NAME_WE)
	set(ontologyVersionFile "${${prefix}_DESTINATION}/versions/${ontologyName}/${${prefix}_VERSION}")
	set(ontologyInstallPath "${${prefix}_DESTINATION}/${${prefix}_FILE_NAME}")
	
	# if the version file exists, file download is skipped.
	# else any existing version of the ontology will be replaced.
	if (NOT EXISTS "${ontologyVersionFile}")
		message(STATUS "Downloading ontology ${ontologyName}-${${prefix}_VERSION}.")
		file(REMOVE_RECURSE "${ontologyInstallPath}")
		file(REMOVE_RECURSE "${${prefix}_DESTINATION}/versions/${ontologyName}")
		
		#
		file(DOWNLOAD
			"${${prefix}_URL}"
			"${ontologyInstallPath}"
			STATUS status)
		list(GET status 0 error_code)
		if( error_code )
			file(REMOVE "${ontologyInstallPath}")
            list(GET status 1 error_message)
            message(FATAL_ERROR "Failed to download ontology ${${prefix}_URL}: ${error_message}" )
        endif()
		
		# store the ontology version.
		# this is done by creating an empty file whose name is the version string.
		file(WRITE "${ontologyVersionFile}" "")
	endif()
endmacro()
