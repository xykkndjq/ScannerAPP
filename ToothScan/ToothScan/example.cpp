

#include "plyio.h"

int main(int argc, char *argv[])
{
	orth::MeshModel mm;
	tinyply::plyio io;



	
	//读ply文件 输入文件名，输出模型文件
	//read_ply_file("example_cube-binary.ply");
	io.read_ply_file(argv[1], mm);

	//写ply文件 输入文件名，输入模型文件，输入是否二进制写的标识
	//write_ply_example("example_cube");
	io.write_ply_example(argv[2], mm,false );
	
    return EXIT_SUCCESS;
}
