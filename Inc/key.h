/* Header file for the key class
 * Key is used to add listeners: on click/on release
 * They also tell us if the key is held
 */

typedef struct {
	GPIO_TypeDef* pin_group;
	uint16_t pin;

	uint8_t is_ready;
	uint8_t is_clicked;
} Key;
