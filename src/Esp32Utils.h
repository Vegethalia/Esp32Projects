#include "esp_partition.h"

//prints the partition table in log_d
void PrintPartitions()
{
	size_t ul;
	esp_partition_iterator_t _mypartiterator;
	const esp_partition_t *_mypart;
	ul = spi_flash_get_chip_size(); 
	log_d("Flash chip size: %u", ul); 
	log_d("Partiton table: (type | subtype | address | size | label | encrypted)");
	_mypartiterator = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);
	if(_mypartiterator) {
		do {
			_mypart = esp_partition_get(_mypartiterator);
			log_d("%4s - %3d - %8d - %8d - %10s - %2i", _mypart->type?"data":"app", _mypart->subtype, _mypart->address, _mypart->size, _mypart->label, _mypart->encrypted);
		} while ((_mypartiterator = esp_partition_next(_mypartiterator)));
	}
	esp_partition_iterator_release(_mypartiterator);
	_mypartiterator = esp_partition_find(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, NULL);
	if(_mypartiterator) {
		do {
			_mypart = esp_partition_get(_mypartiterator);
			log_d("%4s - %3d - %8d - %8d - %10s - %2i", _mypart->type?"data":"app", _mypart->subtype, _mypart->address, _mypart->size, _mypart->label, _mypart->encrypted);
		} while ((_mypartiterator = esp_partition_next(_mypartiterator)));
	}
	esp_partition_iterator_release(_mypartiterator);
}

//Prints the total/free heap memory and the total/free PSRAM (if available) in log_d
void PrintMemoryInfo()
{
	log_d("Total heap: %d", ESP.getHeapSize());
  log_d("Free heap: %d", ESP.getFreeHeap());
  log_d("Total PSRAM: %d", ESP.getPsramSize());
  log_d("Free PSRAM: %d", ESP.getFreePsram());
}
