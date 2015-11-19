# sfmvizcode

Build instructions:<br>
In the project root,<br>
1. mkdir build<br>
2. cd build<br>
3. cmake ..<br>
4. make<br>

Run instructions:<br>
<pre>./viz    path/to/3D/points.ply    path/to/camera/points.ply    path/to/camera/orientation/file</pre><br>
<br>
Example:<br>
Inside the build directory,
<pre>./viz    ../data/option-0000-2.ply    ../data/centers-all-2.ply    ../data/cameras_v2-2.txt</pre><br>

We've included some sample data in PROJECT_ROOT/data
