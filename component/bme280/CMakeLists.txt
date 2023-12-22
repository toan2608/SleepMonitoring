set(app_src bme.c bme280_support.c bme280.c)
set(pre_req log driver)        # log or driver is component of SDK
idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
                    REQUIRES ${pre_req})


# idf_component_register(SRCS "bme280.c" "bme.c" "bme280_support.c" 
#                     INCLUDE_DIRS "include")
