/*
#include <Arduino.h>
#include "esp_partition.h"

void setup()
{
 	Serial.begin(115200);
  // wait for serial monitor to open
  while(!Serial);
}

void loop()
{
	size_t ul;
	esp_partition_iterator_t _mypartiterator;
	const esp_partition_t *_mypart;
	ul = spi_flash_get_chip_size(); Serial.print("Flash chip size: "); Serial.println(ul);
	Serial.println("Partiton table:");
	_mypartiterator = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);
	if(_mypartiterator) {
		do {
			_mypart = esp_partition_get(_mypartiterator);
			printf("%x - %x - %x - %x - %s - %i\r\n", _mypart->type, _mypart->subtype, _mypart->address, _mypart->size, _mypart->label, _mypart->encrypted);
		} while ((_mypartiterator = esp_partition_next(_mypartiterator)));
	}
	esp_partition_iterator_release(_mypartiterator);
	_mypartiterator = esp_partition_find(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, NULL);
	if(_mypartiterator) {
		do {
			_mypart = esp_partition_get(_mypartiterator);
			printf("%x - %x - %x - %x - %s - %i\r\n", _mypart->type, _mypart->subtype, _mypart->address, _mypart->size, _mypart->label, _mypart->encrypted);
		} while ((_mypartiterator = esp_partition_next(_mypartiterator)));
	}
	esp_partition_iterator_release(_mypartiterator);
	
	while(true) {
		delay(400);
	}
}
*/