#include "AtomicPrimitives.h"
#include <assert.h>

#define UNSIGNED_MAX_QUANTIZED_VALUE(bits) ((1<<(bits))-1)
#define SIGNED_MAX_QUANTIZED_VALUE(bits) ((1<<(bits-1))-1)

#define QUANTIZED_VALUE_BITMASK(bits) UNSIGNED_MAX_QUANTIZED_VALUE(bits)

#define QUANTIZE_UNSIGNED_UNIT_VALUE(x, bits) ((uint32_t)((x)*UNSIGNED_MAX_QUANTIZED_VALUE(bits)))&QUANTIZED_VALUE_BITMASK(bits)
#define QUANTIZE_SIGNED_UNIT_VALUE(x, bits) ((uint32_t)((x)*SIGNED_MAX_QUANTIZED_VALUE(bits)))&QUANTIZED_VALUE_BITMASK(bits)

AtomicUnitVector3f::AtomicUnitVector3f()
{
	m_quantizedUnitVector.store(0);
}

AtomicUnitVector3f::AtomicUnitVector3f(const PSVRVector3f &v)
{
	set(v);
}

void AtomicUnitVector3f::set(const PSVRVector3f &v)
{
	uint64_t quantized_unit_vector= 0;

	PSVRVector3f n= PSVR_Vector3fNormalizeWithDefault(&v, k_PSVR_float_vector3_zero);
	uint64_t quantized_x= QUANTIZE_UNSIGNED_UNIT_VALUE(n.x, 21);
	uint64_t quantized_y= QUANTIZE_UNSIGNED_UNIT_VALUE(n.y, 21);
	uint64_t quantized_z= QUANTIZE_UNSIGNED_UNIT_VALUE(n.z, 21);

	quantized_unit_vector= (quantized_z << (21+21)) | (quantized_y << 21) | quantized_x;

	m_quantizedUnitVector.store(quantized_unit_vector);
}

PSVRVector3f AtomicUnitVector3f::get() const
{
	uint64_t quantized_unit_vector= m_quantizedUnitVector.load();

	uint64_t bitmask= QUANTIZED_VALUE_BITMASK(21);
	int64_t quantized_x= (int64_t)(quantized_unit_vector & bitmask);
	int64_t quantized_y= (int64_t)((quantized_unit_vector >> 21) & bitmask);
	int64_t quantized_z= (int64_t)((quantized_unit_vector >> (21+21)) & bitmask);

	float max_quantized_value= (float)(SIGNED_MAX_QUANTIZED_VALUE(21));
	PSVRVector3f v= {
		(float)quantized_x / max_quantized_value,
		(float)quantized_y / max_quantized_value,
		(float)quantized_z / max_quantized_value
	};
	PSVRVector3f n= PSVR_Vector3fNormalizeWithDefault(&v, k_PSVR_float_vector3_zero);

	return n;
}

AtomicQuaternionf::AtomicQuaternionf()
{
	set(*k_PSVR_quaternion_identity);
}

AtomicQuaternionf::AtomicQuaternionf(const PSVRQuatf &q)
{
	set(q);
}

void AtomicQuaternionf::set(const PSVRQuatf &q)
{
	uint64_t quantized_quaternion= 0;

	uint64_t quantized_w= QUANTIZE_UNSIGNED_UNIT_VALUE(q.w, 16);
	uint64_t quantized_x= QUANTIZE_UNSIGNED_UNIT_VALUE(q.x, 16);
	uint64_t quantized_y= QUANTIZE_UNSIGNED_UNIT_VALUE(q.y, 16);
	uint64_t quantized_z= QUANTIZE_UNSIGNED_UNIT_VALUE(q.z, 16);

	quantized_quaternion= (quantized_z << 48) | (quantized_y << 32) | (quantized_x << 16) | quantized_w;

	m_quantizedQuaternion.store(quantized_quaternion);
}

PSVRQuatf AtomicQuaternionf::get() const
{
	uint64_t quantized_quaternion= m_quantizedQuaternion.load();

	uint64_t bitmask= QUANTIZED_VALUE_BITMASK(16);
	int64_t quantized_w= (int64_t)(quantized_quaternion & bitmask);
	int64_t quantized_x= (int64_t)((quantized_quaternion >> 16) & bitmask);
	int64_t quantized_y= (int64_t)((quantized_quaternion >> 32) & bitmask);
	int64_t quantized_z= (int64_t)((quantized_quaternion >> 48) & bitmask);

	float max_quantized_value= (float)(SIGNED_MAX_QUANTIZED_VALUE(16));
	PSVRQuatf q= {
		(float)quantized_w / max_quantized_value,
		(float)quantized_x / max_quantized_value,
		(float)quantized_y / max_quantized_value,
		(float)quantized_z / max_quantized_value
	};
	q= PSVR_QuatfNormalizeWithDefault(&q, k_PSVR_quaternion_identity);

	return q;
}