#ifndef _HELPER_H
#define _HELPER_H

void print_path(struct path path_j){
	//print out the path struct that is passed in
	int i;

	//print pos
	for(i=0; i<6; i++){
		printf("Joint %d's position is: %.2f\n", i, path_j.pos[i]);

		if(i < 5)
			printf("Joint %d's velocity is: \n", i, path_j.vel[i]);

		if(i < 4)
			printf("Joint %d's acceleration is: \n", i, path_j.acc[i]);
	}
}

#ifndef OK
#define OK     (EXIT_SUCCESS)
#endif

#ifndef ERROR
#define ERROR    (EXIT_FAILURE)
#endif

#endif /* _HELPER_H */
