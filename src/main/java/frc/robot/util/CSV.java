package frc.robot.util;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.StringWriter;
import java.nio.file.Path;
import java.util.Collections;
import java.util.List;
import java.util.function.Function;

public class CSV<E> {

  private final List<Column<E>> rows;
  private final File file;
  private final String header;

  private record Column<E>(String name, Function<E, ?> getter) {}

  public static <E> Column<E> column(String name, Function<E, ?> getter) {
    return new Column<>(name, getter);
  }

  public CSV(Path path, List<Column<E>> columns) {
    this.file = path.toFile();
    this.rows = Collections.unmodifiableList(columns);
    this.header = columns.stream().map(Column::name).reduce((a, b) -> a + "," + b).orElseThrow();
  }

  public void write(E row) {
    try (var writer = new StringWriter()) {
      if (!file.isFile()) {
        writer.write(header);
        writer.write(System.lineSeparator());
      }
      writer.write(
          rows.stream()
              .map(r -> r.getter().apply(row).toString())
              .reduce((a, b) -> a + "," + b)
              .orElseThrow());
      writer.write(System.lineSeparator());
      try (var fileWriter = new FileWriter(file, true)) {
        fileWriter.write(writer.toString());
      }
    } catch (IOException e) {
      System.err.println("Failed to write to file.");
      e.printStackTrace();
    }
  }
}
