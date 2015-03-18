#include <stdio.h>
#include <stdlib.h>


int main()
{
    FILE* edges;
    FILE* capture;

    edges = fopen("edges.png", "rb");
    capture = fopen("capture.png", "rb");

    /* seek to the 25th byte of each file to get the 
       PNG type */
    fseek(edges, 25, SEEK_SET);
    fseek(capture, 25, SEEK_SET);

    char edge_buffer;
    char capture_buffer;

    fread(&edge_buffer, 1, 1, edges);
    fread(&capture_buffer, 1, 1, capture);

    printf("edge_buffer = %d\n", (int) edge_buffer);
    printf("capture_buffer = %d\n", (int) capture_buffer);

    return 0;
}
