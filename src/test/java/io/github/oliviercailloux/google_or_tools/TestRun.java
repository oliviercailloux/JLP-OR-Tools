package io.github.oliviercailloux.google_or_tools;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.common.collect.ImmutableMap;
import com.google.ortools.linearsolver.MPConstraint;
import com.google.ortools.linearsolver.MPObjective;
import com.google.ortools.linearsolver.MPSolver;
import com.google.ortools.linearsolver.MPSolver.OptimizationProblemType;
import com.google.ortools.linearsolver.MPVariable;

import io.github.oliviercailloux.jlp.MPExamples;
import io.github.oliviercailloux.jlp.elements.ComparisonOperator;
import io.github.oliviercailloux.jlp.elements.Constraint;
import io.github.oliviercailloux.jlp.elements.SumTerms;
import io.github.oliviercailloux.jlp.elements.Variable;
import io.github.oliviercailloux.jlp.export.Exporter;
import io.github.oliviercailloux.jlp.mp.IMP;
import io.github.oliviercailloux.jlp.mp.MP;
import io.github.oliviercailloux.jlp.mp.MPBuilder;
import io.github.oliviercailloux.jlp.or_tools.OrToolsSolver;
import io.github.oliviercailloux.jlp.result.Result;
import io.github.oliviercailloux.jlp.result.ResultStatus;
import io.github.oliviercailloux.jlp.result.Solution;

class TestRun {

	@SuppressWarnings("unused")
	private static final Logger LOGGER = LoggerFactory.getLogger(TestRun.class);

	/**
	 * Adapted from https://developers.google.com/optimization/introduction/using
	 *
	 */
	@Test
	void testManual() {
		LOGGER.info("Loading native library jniortools (using {}).", System.getProperty("java.library.path"));
		System.loadLibrary("jniortools");

		LOGGER.info("Running the stuff.");

		final MPSolver solver = new MPSolver("LinearExample", OptimizationProblemType.GLOP_LINEAR_PROGRAMMING);
		final double infinity = MPSolver.infinity();

		// x is a continuous non-negative variable
		final MPVariable x = solver.makeNumVar(0.0, infinity, "x");

		// Maximize x
		final MPObjective objective = solver.objective();
		objective.setCoefficient(x, 1);
		objective.setMaximization();

		// x <= 10
		final MPConstraint c = solver.makeConstraint(-infinity, 10.0);
		c.setCoefficient(x, 1);

		solver.solve();

		assertEquals(1, solver.numVariables());
		assertEquals(1, solver.numConstraints());
		assertEquals(10, solver.objective().value());
		assertEquals(10, x.solutionValue());
	}

	@Test
	void testProvided() {
		final IMP mp = MPExamples.getIntOneFourThree().build();
		final Result result = new OrToolsSolver().solve(mp);
		assertEquals(ResultStatus.OPTIMAL, result.getResultStatus());
		final Optional<Solution> solutionOpt = result.getSolution();
		assertTrue(solutionOpt.isPresent());
		final Solution solution = solutionOpt.get();
		final Solution expected = MPExamples.getIntOneFourThreeSolution();
		LOGGER.info("Solution obtained:\n{}.", Exporter.solutionToString(solution));
		assertEquals(expected, solution);
	}

	@Test
	void testTwoConstraintsSameDescription() throws Exception {
		final Variable x = Variable.real("x");
		final Variable y = Variable.real("y");

		final MPBuilder builder = MP.builder();
		builder.addConstraint(Constraint.of("c1", SumTerms.of(1, x), ComparisonOperator.EQ, 2d));
		builder.addConstraint(Constraint.of("c1", SumTerms.of(1, y), ComparisonOperator.EQ, 3d));

		final Result result = new OrToolsSolver().solve(builder);
		assertEquals(ResultStatus.OPTIMAL, result.getResultStatus());
		final Optional<Solution> solutionOpt = result.getSolution();
		assertTrue(solutionOpt.isPresent());
		final Solution solution = solutionOpt.get();
		final Solution expected = Solution.of(builder, 0d, ImmutableMap.of(x, 2d, y, 3d));
		LOGGER.info("Solution obtained:\n{}.", Exporter.solutionToString(solution));
		assertEquals(expected, solution);
	}

}
