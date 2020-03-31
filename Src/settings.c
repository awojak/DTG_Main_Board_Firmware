#include <string.h>
#include <stdint.h>

#include "common/string_light.h"
#include "common/utils.h"

#include "main.h"
#include "settings.h"
#include "common/maths.h"
#include "parameters.h"

void settingGetName(const setting_t *val, char *buf)
{
	strcpy(buf,val->name);
}

bool settingNameContains(const setting_t *val, char *buf, const char *cmdline)
{
	settingGetName(val, buf);
	return strstr(buf, cmdline) != NULL;
}

bool settingNameIsExactMatch(const setting_t *val, char *buf, const char *cmdline, uint8_t var_name_length)
{
	settingGetName(val, buf);
	return sl_strncasecmp(cmdline, buf, strlen(buf)) == 0 && var_name_length == strlen(buf);
}

const setting_t *settingFind(const char *name)
{
	char buf[PARAMETERS_MAX_NAME_LENGTH];
	for (int ii = 0; ii < paramsTableLen; ii++) {
		const setting_t *setting = parametersTable[ii];
		settingGetName(setting, buf);
		if (strcmp(buf, name) == 0) {
			return setting;
		}
	}
	return NULL;
}

const setting_t *settingGet(unsigned index)
{
	return index < paramsTableLen ? parametersTable[index] : NULL;
}

//unsigned settingGetIndex(const setting_t *val)
//{
	//return val - settingsTable;
//}

bool settingsValidate(unsigned *invalidIndex)
{
	for (unsigned ii = 0; ii < paramsTableLen; ii++) {
		const setting_t *setting = settingGet(ii);
		uint32_t min = settingGetMin(setting);
		uint32_t max = settingGetMax(setting);
		void *ptr = settingGetValuePointer(setting);
		bool isValid = false;
		switch (SETTING_TYPE(setting)) {
		case VAR_UINT8:
		{
			uint8_t *value = ptr;
			isValid = *value >= min && *value <= max;
			break;
		}
		case VAR_INT8:
		{
			int8_t *value = ptr;
			isValid = *value >= min && *value <= (int8_t)max;
			break;
		}
		case VAR_UINT16:
		{
			uint16_t *value = ptr;
			isValid = *value >= min && *value <= max;
			break;
		}
		case VAR_INT16:
		{
			int16_t *value = ptr;
			isValid = *value >= min && *value <= (int16_t)max;
			break;
		}
		case VAR_UINT32:
		{
			uint32_t *value = ptr;
			isValid = *value >= (uint32_t)min && *value <= max;
			break;
		}
		case VAR_FLOAT:
		{
			float *value = ptr;
			isValid = *value >= min && *value <= max;
			break;
		}
		case VAR_STRING:
			// We assume all strings are valid
			isValid = true;
			break;
		}
		if (!isValid) {
			if (invalidIndex) {
				*invalidIndex = ii;
			}
			return false;
		}
	}
	return true;
}

size_t settingGetValueSize(const setting_t *val)
{
	switch (SETTING_TYPE(val)) {
		case VAR_UINT8:
			/* fall through */
		case VAR_INT8:
			return 1;
		case VAR_UINT16:
			/* fall through */
		case VAR_INT16:
			return 2;
		case VAR_UINT32:
			/* fall through */
		case VAR_FLOAT:
			return 4;
		case VAR_STRING:
			return settingGetMax(val);
	}
	return 0; // Unreachable
}

void *settingGetValuePointer(const setting_t *val)
{
	return (void *)val->parameterPointer;
}

//const void * settingGetCopyValuePointer(const setting_t *val)
//{
//    const pgRegistry_t *pg = pgFind(settingGetPgn(val));
//    return pg->copy + getValueOffset(val);
//}

int32_t settingGetMin(const setting_t *val)
{
	if (SETTING_MODE(val) == MODE_LOOKUP) {
		return 0;
	}
	return val->config.min;
}

uint32_t settingGetMax(const setting_t *val)
{
	if (SETTING_MODE(val) == MODE_LOOKUP) {
		//TODO nie jestem pewny co do wartosci zwracanej
		return val->lookupTable.valueCount - 1;
	}
	return val->config.max;
}

const lookupTableEntry_t * settingLookupTable(const setting_t *val)
{
	if (SETTING_MODE(val) == MODE_LOOKUP) {
		return &val->lookupTable;
	}
	return NULL;
}

const char * settingLookupValueName(const setting_t *val, unsigned v)
{
	const lookupTableEntry_t *table = settingLookupTable(val);
	if (table && v < table->valueCount) {
		return table->values[v];
	}
	return NULL;
}

size_t settingGetValueNameMaxSize(const setting_t *val)
{
	size_t maxSize = 0;
	const lookupTableEntry_t *table = settingLookupTable(val);
	if (table) {
		for (unsigned ii = 0; ii < table->valueCount; ii++) {
			maxSize = MAX(maxSize, strlen(table->values[ii]));
		}
	}
	return maxSize;
}

const char * settingGetString(const setting_t *val)
{
	if (SETTING_TYPE(val) == VAR_STRING) {
		return settingGetValuePointer(val);
	}
	return NULL;
}

void settingSetString(const setting_t *val, const char *s, size_t size)
{
	if (SETTING_TYPE(val) == VAR_STRING) {
		char *p = settingGetValuePointer(val);
		size_t copySize = MIN(size, settingGetMax(val));
		memcpy(p, s, copySize);
		p[copySize] = '\0';
	}
}

uint32_t settingGetStringMaxLength(const setting_t *val)
{
	if (SETTING_TYPE(val) == VAR_STRING) {
		//TODO nie jestem pewny co do popranosci dzialania tego
		// Max string length is stored as its max
		return settingGetMax(val);
	}
	return 0;
}

