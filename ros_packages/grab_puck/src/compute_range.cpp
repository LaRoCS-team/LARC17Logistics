float compute_range(float distance_from_puck, float coefficient, bool quadratic)
{
	float range = 0.0;
	// quadratic relation
	if(quadratic)
	{
		range = coefficient * distance_from_puck * distance_from_puck;
	}
	// linear relation
	else
	{
		range = coefficient * distance_from_puck;
	}
	return range;
}
