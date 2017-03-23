####################################################################################################################################
'''
* Team ID:          CC#1822
* Author List:      Anubhav Jain, Rahul Bareja
* Filename:         bridge_decision
* Theme:            Cross A Crater
* Functions:        subset_sum, bridgeDecision, map, run
'''
####################################################################################################################################

###############################################
'''
* Function Name:	subset_sum
* Input:		numbers,sum_given,partial list,empty list 
* Output:		list of required combination
* Logic:		check for all the possible combination to get the required sum
* Example Call:	subset_sum(remaining, target, partial + [n])
'''
############################################

def subset_sum(numbers, target, partial=[],list=[]):
    s = sum(partial)
    # check if the partial sum is equals to target
    if s == target: 
        #print "sum(%s)=%s" % (partial, target)
        list.append(partial)
    if s > target:
        return# if we reach the number why bother to continue

    for i in range(len(numbers)):
        n = numbers[i]
        remaining = numbers[i+1:]
        subset_sum(remaining, target, partial + [n])
    return list

###############################################
'''
* Function Name:	bridgeDecision
* Input:		cavities in bridge 1 ,cavities in bridge 2 , no. of obstacles,list of all possible combimations
* Output:		index of selected combination from list, variable for detection of bridge and conditions 
* Logic:		calculate the score of both bridges and check the feasibilty of the bridge
* Example Call:	        bridgeDecision(2,3,3,list)
'''
############################################

def bridgeDecision(cavitiesB1,cavitiesB2,obsB2,combinations):
    scoreB1 = cavitiesB1*200+100            #evaluating score of bridge 1
    scoreB2 = cavitiesB2*200+100*obsB2      #evaluating score of bridge 2
    if scoreB1>scoreB2:
        for i in range(len(combinations)):
            if len(combinations[i]) == cavitiesB1:
                return i,1
        for i in range(len(combinations)):
            if len(combinations[i]) == cavitiesB2:
                return i,2
    else:
        for i in range(len(combinations)):
            if len(combinations[i]) == cavitiesB2:
                return i,3
        for i in range(len(combinations)):
            if len(combinations[i]) == cavitiesB1:
                return i,4
###############################################
'''
* Function Name:	map
* Input:		digits detected,list of digits giving sum
* Output:		decision array 
* Logic:		if the digit of required combination is presint at the index the make the decision of that index 1
* Example Call:	        map(digits,arr)
'''
############################################
def map(digits,arr):
    decision = [0 for i in range(len(digits))]
    for i in range(len(arr)):
        index = arr.index(digits[i])
        arr[index]='XX'
        decision[index]=1
    return decision
    
    
        
#TODO to implement pickup of boulder, use 2-d matrix, 1's at places to be picked up, 0's at places which are not to be picked up
###############################################
'''
* Function Name:	run
* Input:		cavites in Bridge 1,cavities in Bridge 2,no.of obstacles,digits array,Sum To Be Satisfied
* Output:		map,bridge 
* Logic:		first calculating the array list using subset_sum then calling bridgeDecision function 
* Example Call:	        run(2,3,3,digits,14)
'''
############################################
def run(cavB1,cavB2,obsB2,digits,sumToBeSatisfied):
    arr = subset_sum(digits,sumToBeSatisfied)       #calling subset_sum function
    print arr
    
    try:
        a,c=bridgeDecision(cavB1,cavB2,obsB2,arr)       #calling bridge decision function
        if c == 1 :
            return map(digits,arr[a]),1
        elif c == 2:
            return map(digits,arr[a]),2
        elif c == 3:
            return map(digits,arr[a]),1
        elif c == 4:
            return map(digits,arr[a]),2
    except:
        raise    
        
    
if __name__ == "__main__":
    print run(3,2,2,[7,4,3,5],7)
    
    
