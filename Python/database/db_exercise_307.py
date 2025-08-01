"""
데이터베이스 연습 #307
SQLite CRUD 실습
"""
import sqlite3

def setup_307():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_307 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_307_0', 170.87, 'B'), ('item_307_1', 651.8, 'C'), ('item_307_2', 194.34, 'C'), ('item_307_3', 343.74, 'B'), ('item_307_4', 572.1, 'A'), ('item_307_5', 183.74, 'C'), ('item_307_6', 898.07, 'B')]
    ]
    cursor.executemany(
        'INSERT INTO items_307 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_307()
    cursor = conn.cursor()

    print(f"DB 연습 #307")
    cursor.execute('SELECT * FROM items_307')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_307 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
