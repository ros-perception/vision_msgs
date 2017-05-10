# IDs of elements detected with any nonzero probability.
# Does not have to include all possible object
# ids, the scores for any ids not listed are assumed to be 0.
int64[] ids

# The probabilities or confidence values of the detected elements. Must be
# sorted to correspond with the ids field, above
# 0-1, should sum to 1 or less, must be the same length as ids
float64[] scores