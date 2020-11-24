package org.firstinspires.ftc.teamcode.tnwutil.collections;

/**
 * A POJO for containing three elements as an immutable tuple.
 */
public class Triplet<A, B, C> {

    public final A first;
    public final B second;
    public final C third;

    public Triplet(A first, B second, C third) {

        this.first = first;
        this.second = second;
        this.third = third;

    }

    /**
     * This method will return {@code true} if and only if each element in this tuple returns {@code true}
     * when the element's {@link Object#equals(Object)} method is passed the other tuple's respective element.
     *
     * @param o The {@code Triplet} to compare this one to.
     * @return Whether this {@code Triplet} is equivalent to the passed {@code Triplet}.
     */
    @Override
    public boolean equals(Object o) {

        return (o instanceof Triplet<?, ?, ?>)
                && (first.equals(((Triplet<?, ?, ?>) o).first)
                && second.equals(((Triplet<?, ?, ?>) o).second)
                && third.equals(((Triplet<?, ?, ?>) o).third));

    }

}
