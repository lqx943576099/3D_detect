if(NOT DEFINED OUT)
  message(FATAL_ERROR "OUT not set")
endif()
file(WRITE ${OUT} "Astra Python SDK (relocatable)\n")
file(APPEND ${OUT} "Usage:\n")
file(APPEND ${OUT} "  export PYTHONPATH=\"$ENV{PYTHONPATH}:`pwd`/python\"\n")
file(APPEND ${OUT} "  python3 -c \"import astra_py; print('astra ok')\"\n")


