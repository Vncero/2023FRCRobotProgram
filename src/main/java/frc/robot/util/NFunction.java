package frc.robot.util;

import java.util.Objects;
import java.util.function.Function;

@FunctionalInterface
public interface NFunction<T, R> {
    R apply(T... args);

    default <V> NFunction<T, V> andThen(Function<? super R, ? extends V> after) {
        Objects.requireNonNull(after);
        return (T... args) -> after.apply(apply(args));
    }
}