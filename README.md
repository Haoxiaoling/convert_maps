# convert_maps
-----------------------
## This includes three step we must do to finish the whole work.
>
>* The first step is load the transforms matrixes from the .stt files(project02).
>* The  second step is do convert work(project01).
>* Finally, aim to visualization the transforms trees(project03).
_______________________________
### 1. Load the transforms matrixes and point cloud files.
>
> (1).It must include sttl docu. It define the how to get information from .stt files and generate the transforms matrixes the frame to root frame.Using the fuction loadList() to load the information from .stt files and using the generateFrames() to calculate the rootTransform. Then we have finish our prepare work.
>
> (2).After loading the .stt files and calculating the rootTransform. It can get the point cloud based on the name of every frame. And get the coordination xyz information and generate a point cloud. Then, we can save all of the point cloud as .pcd extension.
>
> (3).Reload the point cloud from .pcd files and do transform work into root frame. And put all point cloud together directly. Then save the complete point cloud into a .pcd file.If need, it will be good to do some filtering.
>
> PS: The have four argvs to input(will show if you didn't input the correct amount of the argvs) . The first argv is the path of the .stt file to load. And the second argv is the .xyz files path.
The third argv is .pcd files path ,and these files are convert from the .xyz files. And maybe it will be useful in the future. And the last argv is a file name which will save the complete point cloud.
_______________________________
### 2. Convert the 3D map to 2D map
>
> (1).Filter the point cloud base on pass through filter in PCL. Get the slice of the map and the convert work just use this slice.
>
> (2).To remove the noise, the statistical filter and the radius filter in PCL will be nice work.
>
> (3).Then do project the point cloud into the xy plane. And save the point cloud into .pcd file.
>
> PS: This also has four argvs. The first is the 3D map path. And the second and the third is the threshold of the slice of the map.And if you don't know the argv should be, you can input whatever after execution you can get the scale of the point cloud and the next time, you can input the correct threshold. And the last argv is the path you want to save the 2D map point cloud(It will be a .pcd file).
_______________________________
### 3.Generate the transfome tree.
>
> Coming soon.
