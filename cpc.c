/* Command Process Client program */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/uio.h>

#define	CPC_MSG_LEN		93

char	CPC_TARGET_IP[4] = {192,168,17,213} ; // vwpc10 192.168.17.212
int debug = 0;

int main() {

	int		cpc;
	int		j,len[8];
	int		i,d,c;
	int		ii,dd,cc;
	int		flag = 0;
	int		point;
	int		sp;
	int		str_len;
	char	input[512];
	char	buff[127];
	int		cpc_flag[10];
	char	cpc_com[16];
	char	cpc_narg;
	long long	cpc_arg_i[8];// 64bit integer
	double	cpc_arg_d[8];
	char	cpc_arg_c[8][8];
	unsigned char	cpc_arg_type[8];
	unsigned char	*cpc_msg;

	if (!debug) { cpc = open("/dev/cpc", O_WRONLY); }

	// allocate space to contain the cpc pipeline message
	cpc_msg = (unsigned char *)malloc(CPC_MSG_LEN);

	// shell input loop
	while (1) {
		
		// reset values of cpc-related pipline
		memset(cpc_com, 0x00, sizeof(cpc_com));
		memset(cpc_arg_c, 0x00, sizeof(cpc_arg_c));
		memset(input, 0x00, sizeof(input));
		
		for (j = 0; j < 10; j++) { cpc_flag[j] = 0; }
		cpc_flag[0] = 1;
		
		for (j = 0; j < 8; j++) {
			cpc_arg_i[j] = 0;
			cpc_arg_d[j] = 0.0;
			cpc_arg_type[j] = 0;
		}
		
		flag = 0;
		cpc_narg = 0;
		i = 0; ii = 0; d = 0; dd = 0; c = 0; cc = 0;
		
		// print input prompt and get user input
		printf("\nCPC > ");
		fgets(input,511,stdin);
		
		str_len = strlen(input);
		
		// loop through entire input string, and copy command string into cpc_com
		for (j = 1; j < str_len + 1; j++) {
			if (cpc_flag[flag]) {
				if (input[j] == 32 || input[j] == 40 || input[j] == 0) {
					len[0] = j;
					point = j + 1;	// point marks the start of the next argument
					memcpy(cpc_com,input,len[0]);
					cpc_flag[flag] = 0;
					if (input[j] != 0) { cpc_flag[1] = 1; }
				}
			} // end cpc_flag check if
		} // end str loop

		// loop through rest of input and copy arguments
		for (flag = 1; flag < 9; flag++) {
			sp = 1;

			// if the argument exists
			if (cpc_flag[flag]) {

				// point represents current point in string
				for (j = point; j < str_len + 1; j++) {

					// determine the type of the input by the first character which identifies it
					// type 2 indicates double, type 3 indicates string
					if (cpc_arg_type[flag] == 0) {
						if (input[j] == 34) { cpc_arg_type[flag] = 3; }
						if (input[j] == 46) { cpc_arg_type[flag] = 2; }
					}

					// ignore open paren
					if (input[j] == 40) { point++; } // increment point upon completion of argument

					// edge case for empty arguments
					if ((sp == 1) && (input[j] == 32) || (sp == 1) && (input[j] == 44) ) {
						point++;
					} else { sp = 0; }

					// main case for when argument is more than one character
					if (((sp == 0) && (input[j] == 32)) || ((sp == 0) && (input[j] == 44)) || input[j] == 0 || input[j] == 41) {
						len[flag] = j - point;
						
						// copy argument to cpc_arg_c in case of string argument
						if (cpc_arg_type[flag] == 3) {
							memcpy(cpc_arg_c[c],&input[point+1],len[flag]-2);
							c++; // increment count of string arguments
						} 

						// copy argument to cpc_arg_d in case of float argument
						else if (cpc_arg_type[flag] == 2) {
							memset(buff,0x00,sizeof(buff));
							memcpy(buff,&input[point],len[flag]);
							cpc_arg_d[d] = atof(buff);
							d++; // increment count of double arguments
						} 

						// copy argument to cpc_arg_i in case of integer argument
						else if (len[flag] != 0) {
							cpc_arg_type[flag] = 1;
							memset(buff,0x00,sizeof(buff));
							memcpy(buff,&input[point],len[flag]);
							cpc_arg_i[i] = atoi(buff);
							i++; // increment count of integer arguments
						} 

						// if none of the previous apply, then the argument doesn't exist
						else {
							cpc_flag[flag] = 0;
							break;
						} // end flag check if-else

						// increment argument count and point counters
						cpc_narg++;
						point += len[flag];
						point++;

						cpc_flag[flag] = 0;
						if (input[j] == 32 || input[j]  == 44) { cpc_flag[flag+1] = 1; }
						break;
					} // end escape character check if
				} // end str for loop
			} // end cpc_flag check if
		} // end flag for loop
		
		// break for illegal number of arguments
		if (cpc_narg > 8 || cpc_narg < 0) { break; }
		
		// debug loop printing out input arguments
		if (debug) {
			printf("\n command = %s
				\n cpc_narg = %d\n",cpc_com,cpc_narg);
			
			for (j = 1; j < 9; j++) {
				if (cpc_arg_type[j] == 1) {
					printf("\n cpc_arg_type[%d] = int\n cpc_arg_i[%d] = %d\n",j,j,cpc_arg_i[ii]);
					ii++;
				} else if (cpc_arg_type[j] == 2) {
					printf("\n cpc_arg_type[%d] = double\n cpc_arg_d[%d] = %f\n",j,j,cpc_arg_d[dd]);
					dd++;
				} else if (cpc_arg_type[j] == 3) {
					printf("\n cpc_arg_type[%d] = char *\n cpc_arg_c[%d] = %s\n",j,j,cpc_arg_c[cc]);
					cc++;
				} else break;
			}

		} // end debug
		
		// hardcode exit code
		if (!(strncmp(cpc_com , "exit" , 4))) { break; }
		
		/* Begin CPC pipeline message passing */

		// copy command argument count to cpc pipeline, in the expected format
		memset(cpc_msg,0x00,sizeof(CPC_MSG_LEN));
		memcpy(&cpc_msg[ 0], CPC_TARGET_IP,  4);
		memcpy(&cpc_msg[ 4], cpc_com	  , 16);
		memcpy(&cpc_msg[20], &cpc_narg	  ,  1);
		
		ii = 0; dd = 0; cc = 0;
		
		// copy arguments to cpc pipeline, in the expected format (9 bytes per argument)
		for (j = 0; j < cpc_narg; j++) {

			// one byte indicating the type of each argument
			memcpy(&cpc_msg[21+(9*j)], &cpc_arg_type[j+1] ,  1);

			// eight bytes containing each argument
			if (cpc_arg_type[j+1] == 1) {
				memcpy(&cpc_msg[22+(9*j)], &cpc_arg_i[ii],  8);
				ii++;
			}
			else if (cpc_arg_type[j+1] == 2) {
				memcpy(&cpc_msg[22+(9*j)], &cpc_arg_d[dd],  8);
				dd++;
			}
			else if (cpc_arg_type[j+1] == 3) {
				memcpy(&cpc_msg[22+(9*j)], cpc_arg_c[cc] ,  8);
				cc++;
			} // else memset(&cpc_msg[22+(9*j)], 0x00, 8);
		} // end cpc_narg loop
		
		// debug line which prints full message copied to CPC pipeline
		if (debug) { for (j = 0; j < CPC_MSG_LEN; j++) { printf("cpc_msg[%2d] = %d\n",j,cpc_msg[j]); }}
		
		/* End CPC pipeline message passing */
		
		if (!debug) {
			if (write(cpc,cpc_msg,CPC_MSG_LEN) != CPC_MSG_LEN) printf("CPC > ... ERROR!\n");
			else printf("CPC > ... OK\n");
		}

		// hardcode to finish simulation/operation
		if (!(strncmp(cpc_com , "fin" ,3)))  break;

	} // end while (1)
	
	// close pipeline and free any allocated memory
	close(cpc);
	free(cpc_msg);
}

/* �R�}���h�Ɋւ��钍�ӎ���

�E64bit int�Ƃ���long long�^�Ȃ�Ă������̂��g�p���Ă��܂��B
�@�ilong long�^��C++�ł�64bit int�^�̈�ʓI�Ȍ^���A�炵���ł��j
�@qcc��64bit int��錾����X�}�[�g�Ȃ������킩��Ȃ���������ł��B
�@���ɂ��64bit int�̌^�͕ς��Ǝv���̂ŁA
�@�ł��邾�������A�X�}�[�g�ŃG���K���g�Ȃ�����T���Ă��������B
�@gcc���g���̂ł����Int64���g�����͂��ł��B
�Eexit�Ɠ��͂���ƃv���O�������I�����邱�ƂȂ�cpc�������I�����܂��B
�@�v���O�����{�̂Ƀf�[�^�͑��M����܂���B
�@cpc���I���������Ƃ��v���O�����{�̂��m�邽�߂ɂ́A
�@cp�f�o�C�X��o�^����Ƃ���io_close�֐������삵�Ă��������B
�Eexit�Ŏn�܂�R�}���h����͂����"exit"�ƔF������܂��B
�Eexit�ŏI�����Ă��A�Ă�cpc�𗧂��グ��Γ����͂��ł��B
�Efin�Ŏn�܂�R�}���h����͂����"fin"�Ƃ��ĔF������A
�@�v���O�����{�̂��I�����Acpc���I�����܂��B
�@�f�[�^�͑��M����܂��B
�E�ϐ��Ƃ��ĂW�����ȏ�̕��������͂����ꍇ�A
�@�ŏ��̂W���������͕ϐ��Ƃ��đ�����n�Y�ł��B
�E�T�P�P�����ȍ~�̓��̓f�[�^�͔j������܂��B
�@�o�b�t�@�I�[�o�[�t���[�΍�ł��B

*/

/* ���͂����f�[�^��ϐ��Ɏ�荞�ނƂ���Aunion���g���΁A8byte���ʂ̕ϐ���
memcpy����Ȃ�Ĕ������Ȃ����Ƃ����Ȃ��Ă��ςނ̂ɁA
��arcnet�̃h���C�o���������Ƃ��ɂ��v���܂������A
�ʓ|�Ȃ̂ŁA�㐶�̐l�����A�撣���Ă��������B
union�Ő錾����Ƃ���8byte�ϐ����g��Ȃ��ƁA���o�����ςɂȂ�܂��Bint�����ł����B
�����o���̎���8byte�ϐ���cast���Ă��ǂ������B�R���p�C���ɓ{����\����B
���ƁAunion�ȕϐ���錾������ɁA������malloc����memset��0�ɂ��Ȃ��ƃ��o�C�B
���ꂩ��o�b�t�@�I�[�o�[�t���[�Ƃ��ɂ��C��t���Ă��������B
�ꉞ�A����炵�����Ƃ͂��������ł��B*/
