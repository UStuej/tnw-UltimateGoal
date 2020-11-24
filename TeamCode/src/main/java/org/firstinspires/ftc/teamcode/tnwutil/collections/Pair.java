package org.firstinspires.ftc.teamcode.tnwutil.collections;

/**
 * A POJO for containing two elements as an immutable tuple.
 */
public class Pair<A, B> {

    public final A first;
    public final B second;

    public Pair(A first, B second) {

        this.first = first;
        this.second = second;

    }

    /**
     * This method will return {@code true} if and only if each element in this tuple returns {@code true}
     * when the element's {@link Object#equals(Object)} method is passed the other tuple's respective element.
     *
     * @param o The {@code Pair} to compare this one to.
     * @return Whether this {@code Pair} is equivalent to the passed {@code Pair}.
     */
    @Override
    public boolean equals(Object o) {

        return (o instanceof Pair<?, ?>)
                && (first.equals(((Pair<?, ?>) o).first)
                && second.equals(((Pair<?, ?>) o).second));

    }

}
