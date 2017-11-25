INCLUDE_INSTALL_PATH=/usr/local/include
LIBRARY_INSTALL_PATH=/usr/local/lib

LIBRARY_NAME=MPL


all: install


setup:


install:
	@mkdir -p $(INCLUDE_INSTALL_PATH)/$(LIBRARY_NAME)/
	@cp -r ./include/* $(INCLUDE_INSTALL_PATH)/$(LIBRARY_NAME)/


uninstall:
	@rm -rf $(INCLUDE_INSTALL_PATH)/$(LIBRARY_NAME)


clean:
	@rm -rf ./bin


test: testLagrange testCurl


testLagrange:
	g++ -march=native -Os -std=c++1z -I./include ./src/lagrange.cpp -o ./bin/lagrange

testCurl:
	g++ -march=native -Os -std=c++1z -I/usr/local/include -I./include ./src/curl.cpp -L/usr/local/lib -lcurl -o ./bin/curl

